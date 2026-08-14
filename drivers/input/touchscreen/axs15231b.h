/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _AXS15231B_H
#define _AXS15231B_H

#include <linux/errno.h>
#include <linux/types.h>

#define AXS15231B_REPORT_LEN	8
#define AXS15231B_MAX_TOUCHES	1

struct axs15231b_point {
	unsigned int x;
	unsigned int y;
	bool active;
};

static inline int axs15231b_parse_report(const u8 *report, size_t report_len,
					 struct axs15231b_point *point)
{
	u8 touches;

	if (!report || !point)
		return -EINVAL;
	if (report_len != AXS15231B_REPORT_LEN)
		return -EMSGSIZE;

	touches = report[1];
	if (touches > AXS15231B_MAX_TOUCHES)
		return -EPROTO;

	point->active = touches == 1;
	if (!point->active) {
		point->x = 0;
		point->y = 0;
		return 0;
	}

	point->x = ((report[2] & 0x0f) << 8) | report[3];
	point->y = ((report[4] & 0x0f) << 8) | report[5];

	return 0;
}

#endif /* _AXS15231B_H */
