/*
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 */

/*
 * Copyright 2025 Tintri by DDN, Inc. All rights reserved.
 */

#include <sys/types.h>
#include <sys/sunddi.h>
#include <sys/rwlock.h>
#include <sys/bootstat.h>
#include <sys/kobj.h>
#include <sys/nvpair.h>

#include "ntb_impl.h"

#define	isspace(ch)	((ch) == ' ' || (ch) == '\r' || (ch) == '\n' || \
			(ch) == '\t' || (ch) == '\f')

#define	MAX_NAMES_FILE_SIZE	(32 * 1024 - 1)	/* 32KB - 1 (for NULL char) */

static krwlock_t	names_lock;

static boottime_t	last_change_time = { 0, 0 };

static  char		names_file[] = "/etc/ntb_transport/names_to_endpoint";

static nvlist_t		*ep_nvlist;

void
ntb_names_init(void)
{
	VERIFY(nvlist_alloc(&ep_nvlist, NV_UNIQUE_NAME, KM_SLEEP) == 0);
	rw_init(&names_lock, NULL, RW_DRIVER, NULL);
}

void
ntb_names_fini(void)
{
	nvlist_free(ep_nvlist);
	rw_destroy(&names_lock);
}

static void
empty_ep_nvlist(void)
{
	nvpair_t	*pair, *next;

	pair = nvlist_next_nvpair(ep_nvlist, NULL);
	while (pair) {
		next = nvlist_next_nvpair(ep_nvlist, pair);
		(void) nvlist_remove_nvpair(ep_nvlist, pair);
		pair = next;
	}
}

/*
 * If the names_to_endpoint file has changed, it will read the contents
 * and rebuild the nv_list.
 * Will exit with names_lock held as a *reader*
 * The file has a simple format:
 *	- a '#' at the beginning of a line is a comment
 *	- the name of the endpoint and its assigned id appear on
 *	  the same line, separated by white spaces(s).
 *	- a white space after the id followed be text is taken as a comment
 * Eg.
 *	# This is a comment
 *	test-endpoint	254	Used by test driver
 *	perf-endpoint	255	Used by performance module of test driver
 *
 */
static void
read_names_file(void)
{
	struct _buf	*file;
	nvlist_t	*new_nvlist = NULL;
	struct bootstat	bst;
	char		*buffer = NULL;
	char		*line, *token, *next, *end;
	int		linenum, count;
	boolean_t	syntax_err = B_FALSE;
	unsigned long	value;
	krw_t		rw = RW_READER;

	/*
	 * We get the lock as reader first, and if the file hasn't changed
	 * we don't need to do anything. If the file has changed we try
	 * and upgrade the lock to a writer, if that fails we drop the
	 * lock altogether and then start again but as a writer.
	 * Once we have the lock as writer we can update the nvlist.
	 */
retry:
	rw_enter(&names_lock, rw);

	file = kobj_open_file(names_file);
	if (file == (struct _buf *)-1) {
		/*
		 * names_to_endpoint is not there. Make sure the nvlist
		 * is empty
		 */
		if (rw == RW_READER) {
			/* we need to be writer to modify the nvlist */
			if (rw_tryupgrade(&names_lock) == 0) {
				rw = RW_WRITER;
				rw_exit(&names_lock);
				goto retry;
			}
		}

		empty_ep_nvlist();
		rw_downgrade(&names_lock);
		return;
	}

	if (kobj_fstat(file->_fd, &bst) != 0) {
		cmn_err(CE_WARN, "!Failed to stat %s", names_file);
		goto exit;
	}

	if (bst.st_mtim.tv_sec == last_change_time.tv_sec &&
	    bst.st_mtim.tv_nsec == last_change_time.tv_nsec) {
		/* file hasn't changed since we last looked */
		goto exit;
	}

	/* we need to be writer to modify the nvlist */
	if (rw_tryupgrade(&names_lock) == 0) {
		rw = RW_WRITER;
		rw_exit(&names_lock);
		kobj_close_file(file);
		goto retry;
	}

	ASSERT(rw_read_locked(&names_lock) == 0);

	last_change_time = bst.st_mtim;

	if (bst.st_size == 0) {
		/* file is empty, free nvlist */
		empty_ep_nvlist();
		goto exit;
	}

	if (bst.st_size > MAX_NAMES_FILE_SIZE) {
		bst.st_size = MAX_NAMES_FILE_SIZE;
		cmn_err(CE_WARN, "!Contents of %s truncated to %d characters",
		    names_file, MAX_NAMES_FILE_SIZE);
	}

	/* add 1 for null terminator */
	buffer = kmem_alloc(bst.st_size + 1, KM_SLEEP);
	count = kobj_read_file(file, buffer, bst.st_size, 0);
	if (count < 0) {
		/*
		 * failed to read the file for some reason,
		 * leave the nvlist as is.
		 */
		cmn_err(CE_WARN, "!Read of %s failed", names_file);
		goto exit;
	}

	buffer[count] = '\0';

	/*
	 * Now parse the file and re-build the nvlist
	 */
	if (nvlist_alloc(&new_nvlist, NV_UNIQUE_NAME, KM_SLEEP) != 0)
		goto exit;

	linenum = 0;
	for (line = buffer; line; line = next) {
		linenum++;

		next = strchr(line, '\n');
		if (next != NULL)
			*next++ = '\0';

		if (*line == '#') {
			/* skip comments */
			continue;
		}

		/* skip whitespace at start of line */
		while (isspace(*line))
			line++;

		if (!*line) {
			/* blank line */
			continue;
		}

		token = line;

		/* skip to next whitespace character beyond token */
		while (*line && !isspace(*line))
			line++;

		if (!*line) {
			cmn_err(CE_WARN, "!%s (line %d): value expected",
			    names_file, linenum);
			syntax_err = B_TRUE;
			continue;
		}

		/* Null terminate the token */
		*line++ = '\0';

		/* now we are at the value */
		if (ddi_strtoul(line, &end, 10, &value) != 0 || end == line ||
		    (*end && !isspace(*end))) {
			/* value is bad */
			cmn_err(CE_WARN, "!%s (line %d): error parsing value "
			    "for %s", names_file, linenum, token);
			syntax_err = B_TRUE;
			continue;
		}

		if (value < HIGHEST_ENDPOINT_ID) {
			(void) nvlist_add_uint32(new_nvlist, token,
			    (uint32_t)value);
		} else {
			cmn_err(CE_WARN, "!%s (line %d): %s = %lu is too "
			    "large. Must be less than %d", names_file, linenum,
			    token, value, HIGHEST_ENDPOINT_ID);
			syntax_err = B_TRUE;
		}
	}

	if (!syntax_err) {
		nvlist_free(ep_nvlist);
		ep_nvlist = new_nvlist;
		new_nvlist = NULL;
	}

exit:
	kobj_close_file(file);

	if (new_nvlist)
		nvlist_free(new_nvlist);

	if (buffer)
		kmem_free(buffer, bst.st_size + 1);

	if (rw == RW_WRITER)
		rw_downgrade(&names_lock);
}

/*
 * Given an endpoint name, look it up and return its id.
 * If no id is found return -1
 */
int
ntb_name_to_endpoint_id(const char *name)
{
	uint32_t	ep;
	int		rv;

	/* returns holding names_lock */
	read_names_file();

	if (nvlist_lookup_uint32(ep_nvlist, name, &ep) == 0)
		rv = (int)ep;
	else
		rv = -1;

	rw_exit(&names_lock);

	return (rv);
}
