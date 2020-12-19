#ifndef __CMD_H
#define __CMD_H

#include <stdint.h>

#define CMD_FAILED "Command returned error status, Returned code is %d (= %#X)\r\n"
#define CMD_SUCCESS "Command returned normal status\r\n"

typedef struct _console_cmd
{
   char *cmd;
   int8_t (*callfn)(uint8_t argc, char **argv);
   char *desc;
   char *info;
   char *example;
}console_cmd;

extern console_cmd cmdlist[];
extern const uint32_t cmd_count;

int8_t cmd_help(uint8_t argc, char **argv);
int8_t cmd_help2(char *cmd_name);
int8_t process_cmd(const char *buf);

#endif
