// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *   Copyright (C) 2018 Samsung Electronics Co., Ltd.
 */

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/rwsem.h>

#include "ksmbd_ida.h"
#include "user_session.h"
#include "user_config.h"
#include "tree_connect.h"
#include "../transport_ipc.h"
#include "../connection.h"
#include "../vfs_cache.h"

static DEFINE_IDA(session_ida);

#define SESSION_HASH_BITS		3
static DEFINE_HASHTABLE(sessions_table, SESSION_HASH_BITS);
static DECLARE_RWSEM(sessions_table_lock);

struct ksmbd_session_rpc {
	int			id;
	unsigned int		method;
	struct list_head	list;
};

static void free_channel_list(struct ksmbd_session *sess)
{
	struct channel *chann;
	struct channel *tmp;

	down_write(&sess->chann_lock);
	list_for_each_entry_safe(chann, tmp, &sess->ksmbd_chann_list, list) {
		list_del(&chann->list);
		kfree(chann);
	}
	up_write(&sess->chann_lock);
}

static void __session_rpc_close(struct ksmbd_session *sess,
				struct ksmbd_session_rpc *entry)
{
	struct ksmbd_rpc_command *resp;

	resp = ksmbd_rpc_close(sess, entry->id);
	if (!resp)
		pr_err("Unable to close RPC pipe %d\n", entry->id);

	kvfree(resp);
	ksmbd_rpc_id_free(entry->id);
	kfree(entry);
}

static void ksmbd_session_rpc_clear_list(struct ksmbd_session *sess)
{
	struct ksmbd_session_rpc *entry;
	struct ksmbd_session_rpc *tmp;

	down_write(&sess->rpc_lock);
	list_for_each_entry_safe(entry, tmp, &sess->rpc_handle_list, list) {
		list_del_init(&entry->list);
		__session_rpc_close(sess, entry);
	}
	up_write(&sess->rpc_lock);
}

static int __rpc_method(char *rpc_name)
{
	if (!strcmp(rpc_name, "\\srvsvc") || !strcmp(rpc_name, "srvsvc"))
		return KSMBD_RPC_SRVSVC_METHOD_INVOKE;

	if (!strcmp(rpc_name, "\\wkssvc") || !strcmp(rpc_name, "wkssvc"))
		return KSMBD_RPC_WKSSVC_METHOD_INVOKE;

	if (!strcmp(rpc_name, "LANMAN") || !strcmp(rpc_name, "lanman"))
		return KSMBD_RPC_RAP_METHOD;

	if (!strcmp(rpc_name, "\\samr") || !strcmp(rpc_name, "samr"))
		return KSMBD_RPC_SAMR_METHOD_INVOKE;

	if (!strcmp(rpc_name, "\\lsarpc") || !strcmp(rpc_name, "lsarpc"))
		return KSMBD_RPC_LSARPC_METHOD_INVOKE;

	pr_err("Unsupported RPC: %s\n", rpc_name);
	return 0;
}

int ksmbd_session_rpc_open(struct ksmbd_session *sess, char *rpc_name)
{
	struct ksmbd_session_rpc *entry;
	struct ksmbd_rpc_command *resp;
	int method;

	method = __rpc_method(rpc_name);
	if (!method)
		return -EINVAL;

	entry = kzalloc(sizeof(struct ksmbd_session_rpc), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->method = method;
	entry->id = ksmbd_ipc_id_alloc();
	if (entry->id < 0)
		goto free_entry;

	INIT_LIST_HEAD(&entry->list);
	down_read(&sess->rpc_lock);
	list_add(&entry->list, &sess->rpc_handle_list);

	resp = ksmbd_rpc_open(sess, entry->id);
	if (!resp) {
		list_del_init(&entry->list);
		up_read(&sess->rpc_lock);
		goto free_id;
	}

	up_read(&sess->rpc_lock);
	kvfree(resp);
	return entry->id;

free_id:
	ksmbd_rpc_id_free(entry->id);
free_entry:
	kfree(entry);
	return -EINVAL;
}

void ksmbd_session_rpc_close(struct ksmbd_session *sess, int id)
{
	struct ksmbd_session_rpc *entry;

	down_write(&sess->rpc_lock);
	list_for_each_entry(entry, &sess->rpc_handle_list, list) {
		if (entry->id != id)
			continue;

		list_del_init(&entry->list);
		__session_rpc_close(sess, entry);
		up_write(&sess->rpc_lock);
		return;
	}
	up_write(&sess->rpc_lock);
}

int ksmbd_session_rpc_method(struct ksmbd_session *sess, int id)
{
	struct ksmbd_session_rpc *entry;

	lockdep_assert_held(&sess->rpc_lock);
	list_for_each_entry(entry, &sess->rpc_handle_list, list) {
		if (entry->id == id)
			return entry->method;
	}

	return 0;
}

void ksmbd_session_destroy(struct ksmbd_session *sess)
{
	if (!sess)
		return;

	if (sess->user)
		ksmbd_free_user(sess->user);

	ksmbd_tree_conn_session_logoff(sess);
	ksmbd_destroy_file_table(&sess->file_table);
	ksmbd_session_rpc_clear_list(sess);
	free_channel_list(sess);
	kfree(sess->Preauth_HashValue);
	ksmbd_release_id(&session_ida, sess->id);
	kfree(sess);
}

struct ksmbd_session *__session_lookup(unsigned long long id)
{
	struct ksmbd_session *sess;

	hash_for_each_possible(sessions_table, sess, hlist, id) {
		if (id == sess->id) {
			sess->last_active = jiffies;
			return sess;
		}
	}
	return NULL;
}

static void ksmbd_expire_session(struct ksmbd_conn *conn)
{
	struct ksmbd_session *sess;
	struct ksmbd_session *tmp;

	down_write(&conn->session_lock);
	list_for_each_entry_safe(sess, tmp, &conn->sessions, sessions_list) {
		if (sess->state == SMB2_SESSION_VALID &&
		    !time_after(jiffies,
				sess->last_active + SMB2_SESSION_TIMEOUT))
			continue;

		list_del_init(&sess->sessions_list);
#ifdef CONFIG_SMB_INSECURE_SERVER
		if (hash_hashed(&sess->hlist))
			hash_del(&sess->hlist);
#else
		hash_del(&sess->hlist);
#endif
		ksmbd_session_destroy(sess);
	}
	up_write(&conn->session_lock);
}

int ksmbd_session_register(struct ksmbd_conn *conn,
			   struct ksmbd_session *sess)
{
	sess->dialect = conn->dialect;
	memcpy(sess->ClientGUID, conn->ClientGUID, SMB2_CLIENT_GUID_SIZE);
	ksmbd_expire_session(conn);

	down_write(&conn->session_lock);
	list_add(&sess->sessions_list, &conn->sessions);
	up_write(&conn->session_lock);

	return 0;
}

static int ksmbd_chann_del(struct ksmbd_conn *conn, struct ksmbd_session *sess)
{
	struct channel *chann;

	list_for_each_entry(chann, &sess->ksmbd_chann_list, list) {
		if (chann->conn != conn)
			continue;

		list_del_init(&chann->list);
		kfree(chann);
		return 0;
	}

	return -ENOENT;
}

void ksmbd_sessions_deregister(struct ksmbd_conn *conn)
{
	struct ksmbd_session *sess;
	struct ksmbd_session *tmp;

	down_write(&sessions_table_lock);
	if (conn->binding) {
		int bkt;
		struct hlist_node *tmp_hlist;

		hash_for_each_safe(sessions_table, bkt, tmp_hlist, sess, hlist) {
			if (!ksmbd_chann_del(conn, sess) &&
			    list_empty(&sess->ksmbd_chann_list)) {
#ifdef CONFIG_SMB_INSECURE_SERVER
				if (hash_hashed(&sess->hlist))
					hash_del(&sess->hlist);
#else
				hash_del(&sess->hlist);
#endif
				ksmbd_session_destroy(sess);
			}
		}
	}
	up_write(&sessions_table_lock);

	down_write(&conn->session_lock);
	list_for_each_entry_safe(sess, tmp, &conn->sessions, sessions_list) {
		struct channel *chann;

		list_for_each_entry(chann, &sess->ksmbd_chann_list, list) {
			if (chann->conn != conn)
				ksmbd_conn_set_exiting(chann->conn);
		}

		ksmbd_chann_del(conn, sess);
		if (list_empty(&sess->ksmbd_chann_list)) {
			list_del_init(&sess->sessions_list);
#ifdef CONFIG_SMB_INSECURE_SERVER
			if (hash_hashed(&sess->hlist))
				hash_del(&sess->hlist);
#else
			hash_del(&sess->hlist);
#endif
			ksmbd_session_destroy(sess);
		}
	}
	up_write(&conn->session_lock);
}

struct ksmbd_session *ksmbd_session_lookup(struct ksmbd_conn *conn,
					    unsigned long long id)
{
	struct ksmbd_session *sess;

	down_read(&conn->session_lock);
	list_for_each_entry(sess, &conn->sessions, sessions_list) {
		if (sess->id != id)
			continue;

		sess->last_active = jiffies;
		up_read(&conn->session_lock);
		return sess;
	}
	up_read(&conn->session_lock);

	return NULL;
}

struct ksmbd_session *ksmbd_session_lookup_slowpath(unsigned long long id)
{
	struct ksmbd_session *sess;

	down_read(&sessions_table_lock);
	sess = __session_lookup(id);
	if (sess)
		sess->last_active = jiffies;
	up_read(&sessions_table_lock);

	return sess;
}

struct ksmbd_session *ksmbd_session_lookup_all(struct ksmbd_conn *conn,
					       unsigned long long id)
{
	struct ksmbd_session *sess;

	sess = ksmbd_session_lookup(conn, id);
	if (!sess && conn->binding)
		sess = ksmbd_session_lookup_slowpath(id);
	if (sess && sess->state != SMB2_SESSION_VALID)
		sess = NULL;
	return sess;
}

struct preauth_session *ksmbd_preauth_session_alloc(struct ksmbd_conn *conn,
						    u64 sess_id)
{
	struct preauth_session *sess;

	sess = kmalloc(sizeof(struct preauth_session), GFP_KERNEL);
	if (!sess)
		return NULL;

	sess->id = sess_id;
	memcpy(sess->Preauth_HashValue, conn->preauth_info->Preauth_HashValue,
	       PREAUTH_HASHVALUE_SIZE);
	list_add(&sess->preauth_entry, &conn->preauth_sess_table);

	return sess;
}

void destroy_previous_session(struct ksmbd_conn *conn,
			      struct ksmbd_user *user, u64 id)
{
	struct ksmbd_session *prev_sess;
	struct ksmbd_user *prev_user;

	down_write(&sessions_table_lock);
	down_write(&conn->session_lock);
	prev_sess = __session_lookup(id);
	if (!prev_sess || prev_sess->state == SMB2_SESSION_EXPIRED)
		goto out;

	prev_user = prev_sess->user;
	if (!prev_user ||
	    strcmp(user->name, prev_user->name) ||
	    user->passkey_sz != prev_user->passkey_sz ||
	    memcmp(user->passkey, prev_user->passkey, user->passkey_sz))
		goto out;

	ksmbd_destroy_file_table(&prev_sess->file_table);
	prev_sess->state = SMB2_SESSION_EXPIRED;
out:
	up_write(&conn->session_lock);
        up_write(&sessions_table_lock);
}

static bool ksmbd_preauth_session_id_match(struct preauth_session *sess,
					   unsigned long long id)
{
	return sess->id == id;
}

struct preauth_session *ksmbd_preauth_session_lookup(struct ksmbd_conn *conn,
						     unsigned long long id)
{
	struct preauth_session *sess = NULL;

	list_for_each_entry(sess, &conn->preauth_sess_table, preauth_entry) {
		if (ksmbd_preauth_session_id_match(sess, id))
			return sess;
	}
	return NULL;
}

#ifdef CONFIG_SMB_INSECURE_SERVER
static int __init_smb1_session(struct ksmbd_session *sess)
{
	int id = ksmbd_acquire_smb1_uid(&session_ida);

	if (id < 0)
		return -EINVAL;
	sess->id = id;
	return 0;
}
#endif

static int __init_smb2_session(struct ksmbd_session *sess)
{
	int id = ksmbd_acquire_smb2_uid(&session_ida);

	if (id < 0)
		return -EINVAL;
	sess->id = id;
	return 0;
}

static struct ksmbd_session *__session_create(int protocol)
{
	struct ksmbd_session *sess;
	int ret;

	sess = kzalloc(sizeof(struct ksmbd_session), GFP_KERNEL);
	if (!sess)
		return NULL;

	if (ksmbd_init_file_table(&sess->file_table))
		goto error;

	sess->last_active = jiffies;
	sess->state = SMB2_SESSION_IN_PROGRESS;
	set_session_flag(sess, protocol);
	INIT_LIST_HEAD(&sess->tree_conns);
	INIT_LIST_HEAD(&sess->ksmbd_chann_list);
	INIT_LIST_HEAD(&sess->rpc_handle_list);
	INIT_LIST_HEAD(&sess->sessions_list);
	sess->sequence_number = 1;
	rwlock_init(&sess->tree_conns_lock);
	init_rwsem(&sess->chann_lock);
	init_rwsem(&sess->rpc_lock);

	switch (protocol) {
#ifdef CONFIG_SMB_INSECURE_SERVER
	case CIFDS_SESSION_FLAG_SMB1:
		ret = __init_smb1_session(sess);
		break;
#endif
	case CIFDS_SESSION_FLAG_SMB2:
		ret = __init_smb2_session(sess);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret)
		goto error;

	ida_init(&sess->tree_conn_ida);

	down_write(&sessions_table_lock);
	hash_add(sessions_table, &sess->hlist, sess->id);
	up_write(&sessions_table_lock);

	return sess;

error:
	ksmbd_session_destroy(sess);
	return NULL;
}

#ifdef CONFIG_SMB_INSECURE_SERVER
struct ksmbd_session *ksmbd_smb1_session_create(void)
{
	return __session_create(CIFDS_SESSION_FLAG_SMB1);
}
#endif

struct ksmbd_session *ksmbd_smb2_session_create(void)
{
	return __session_create(CIFDS_SESSION_FLAG_SMB2);
}

int ksmbd_acquire_tree_conn_id(struct ksmbd_session *sess)
{
	int id = -EINVAL;

#ifdef CONFIG_SMB_INSECURE_SERVER
	if (test_session_flag(sess, CIFDS_SESSION_FLAG_SMB1))
		id = ksmbd_acquire_smb1_tid(&sess->tree_conn_ida);
#endif
	if (test_session_flag(sess, CIFDS_SESSION_FLAG_SMB2))
		id = ksmbd_acquire_smb2_tid(&sess->tree_conn_ida);

	return id;
}

void ksmbd_release_tree_conn_id(struct ksmbd_session *sess, int id)
{
	if (id >= 0)
		ksmbd_release_id(&sess->tree_conn_ida, id);
}
