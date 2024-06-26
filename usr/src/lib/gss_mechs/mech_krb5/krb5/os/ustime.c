/*
 * lib/krb5/os/ustime.c
 *
 * Copyright 1990,1991 by the Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * Export of this software from the United States of America may
 *   require a specific license from the United States Government.
 *   It is the responsibility of any person or organization contemplating
 *   export to obtain such a license before exporting.
 *
 * WITHIN THAT CONSTRAINT, permission to use, copy, modify, and
 * distribute this software and its documentation for any purpose and
 * without fee is hereby granted, provided that the above copyright
 * notice appear in all copies and that both that copyright notice and
 * this permission notice appear in supporting documentation, and that
 * the name of M.I.T. not be used in advertising or publicity pertaining
 * to distribution of the software without specific, written prior
 * permission.  Furthermore if you modify this software you must label
 * your software as modified software and not distribute it in such a
 * fashion that it might be confused with the original M.I.T. software.
 * M.I.T. makes no representations about the suitability of
 * this software for any purpose.  It is provided "as is" without express
 * or implied warranty.
 *
 * krb5_crypto_us_timeofday() does all of the real work; however, we
 * handle the time offset adjustment here, since this is context
 * specific, and the crypto version of this call doesn't have access
 * to the context variable.  Fortunately the only user of
 * krb5_crypto_us_timeofday in the crypto library doesn't require that
 * this time adjustment be done.
 */

#include "k5-int.h"

krb5_error_code KRB5_CALLCONV
krb5_us_timeofday(krb5_context context, krb5_timestamp *seconds, krb5_int32 *microseconds)
{
    krb5_os_context os_ctx = context->os_context;
    krb5_int32 sec, usec;
    krb5_error_code retval;

    if (os_ctx->os_flags & KRB5_OS_TOFFSET_TIME) {
	    *seconds = os_ctx->time_offset;
	    *microseconds = os_ctx->usec_offset;
	    return 0;
    }
    retval = krb5_crypto_us_timeofday(&sec, &usec);
    if (retval)
	    return retval;
    if (os_ctx->os_flags & KRB5_OS_TOFFSET_VALID) {
	    usec += os_ctx->usec_offset;
	    if (usec > 1000000) {
		    usec -= 1000000;
		    sec++;
	    }
	    if (usec < 0) {
		usec += 1000000;
		sec--;
	    }
	    sec += os_ctx->time_offset;
    }
    *seconds = sec;
    *microseconds = usec;
    return 0;
}
