/*
 * assert.h
 *
 *  Created on: Jan 4, 2018
 *      Author: air
 */

#ifndef __ASSERT_H__
#define __ASSERT_H__

#ifdef __cplusplus
extern "C" {
#endif

void AssertHandler(uint8 assertCause, uint8 assertSubcause);

#ifdef __cplusplus
}
#endif

#endif /* __ASSERT_H__ */
