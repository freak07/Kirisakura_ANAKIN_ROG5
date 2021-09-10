/*****************************************************************************
* File: file.h
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All information contained herein is and remains the property of Sentons
* Incorporated and its suppliers if any. The intellectual and technical
* concepts contained herein are proprietary to Sentons Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law. Dissemination
* of this information or reproduction of this material is strictly forbidden
* unless prior written permission is obtained from Sentons Incorporated.
*
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*****************************************************************************/
#include <linux/fs.h>      /* Needed by filp */
#include <asm/uaccess.h>   /* Needed by segment descriptors */
//#include <asm/segment.h>

#ifndef FILE_H
#define FILE_H

int     file_open(const char* path, int flags, int rights, struct file **f);
void    file_close(struct file* file);
int     file_read(struct file* file,
                 unsigned long long offset,
                 unsigned char* data,
                 unsigned int size);
int     file_write(struct file* file,
                   unsigned long long offset,
                   unsigned char* data,
                   unsigned int size);
int     file_size(struct file *file,
                    int *p_size);


#endif // FILE_H

