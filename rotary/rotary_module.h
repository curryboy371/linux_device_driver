#ifndef __ROTARY_MODULE_H__
#define __ROTARY_MODULE_H__

int rotary_get_count(void);
int rotary_get_toggle(void);
int rotary_get_direction(void);

int rotary_init(void);
void rotary_exit(void);

#endif // __ROTARY_MODULE_H__
