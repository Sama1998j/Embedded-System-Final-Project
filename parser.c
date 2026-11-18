#include "parser.h"
#include "uart.h"
#include "fsm.h"
#include "motor.h"

// Flags defined in main.c, toggled here via $PCLOG,...*
extern int log_batt_enable;
extern int log_dist_enable;
extern int log_angle_enable;
extern int log_gyro_enable;

/* Holds the characters between '$' and '*'. The longest valid payload is
 * 19 characters, so 32 leaves ample margin. */
#define PAYLOAD_SIZE 32

static char payload[PAYLOAD_SIZE];
static int payload_len = 0;
static int receiving = 0;

/* Upper bound on parsed values, well below the 16-bit limit, so an over-long
 * digit string cannot overflow into an in-range value. */
#define PARSE_INT_MAX 10000

/* Parses a signed integer. Returns 1 on success, 0 if empty, non-numeric,
 * or above PARSE_INT_MAX. */
static int parse_int(const char *s, int *value)
{
    int result = 0;
    int sign = 1;

    if (*s == '-')
    {
        sign = -1;
        s++;
    }

    if (*s == '\0')
    {
        return 0;
    }

    while (*s != '\0')
    {
        if (*s < '0' || *s > '9')
        {
            return 0;
        }

        result = result * 10 + (*s - '0');

        if (result > PARSE_INT_MAX)
        {
            return 0;   // too large to be a valid parameter
        }

        s++;
    }

    *value = sign * result;
    return 1;
}

// Handles "BATT,x" / "DIST,x" / "ANGLE,x" / "GYRO,x" (the part of
// $PCLOG,...* after the "PCLOG," prefix) -- enables/disables one report
static void handle_pclog(const char *rest)
{
    int comma = -1;
    int i;
    int value;

    for (i = 0; rest[i] != '\0'; i++)
    {
        if (rest[i] == ',')
        {
            comma = i;
            break;
        }
    }

    if (comma < 0 || !parse_int(&rest[comma + 1], &value) || (value != 0 && value != 1))
    {
        uart1_send_string("$ERR*");
        return;
    }

    if (comma == 4 && rest[0]=='B' && rest[1]=='A' && rest[2]=='T' && rest[3]=='T')
    {
        log_batt_enable = value;
    }
    else if (comma == 4 && rest[0]=='D' && rest[1]=='I' && rest[2]=='S' && rest[3]=='T')
    {
        log_dist_enable = value;
    }
    else if (comma == 5 && rest[0]=='A' && rest[1]=='N' && rest[2]=='G' && rest[3]=='L' && rest[4]=='E')
    {
        log_angle_enable = value;
    }
    else if (comma == 4 && rest[0]=='G' && rest[1]=='Y' && rest[2]=='R' && rest[3]=='O')
    {
        log_gyro_enable = value;
    }
    else
    {
        uart1_send_string("$ERR*");
    }
}

// Handles "THRESHOLD,x" / "MAXSPEED,x" (the part of $PCCFG,...* after
// the "PCCFG," prefix) -- sets a runtime configuration value
static void handle_pccfg(const char *rest)
{
    int comma = -1;
    int i;
    int value;

    for (i = 0; rest[i] != '\0'; i++)
    {
        if (rest[i] == ',')
        {
            comma = i;
            break;
        }
    }

    if (comma < 0 || !parse_int(&rest[comma + 1], &value))
    {
        uart1_send_string("$ERR*");
        return;
    }

    if (comma == 9 && rest[0]=='T' && rest[1]=='H' && rest[2]=='R' && rest[3]=='E' &&
        rest[4]=='S' && rest[5]=='H' && rest[6]=='O' && rest[7]=='L' && rest[8]=='D')
    {
        fsm_set_threshold(value);
    }
    else if (comma == 8 && rest[0]=='M' && rest[1]=='A' && rest[2]=='X' && rest[3]=='S' &&
             rest[4]=='P' && rest[5]=='E' && rest[6]=='E' && rest[7]=='D')
    {
        motor_set_max_speed(value);
    }
    else
    {
        uart1_send_string("$ERR*");
    }
}

// Handles "off" (the part of $PCEMR,off* after the "PCEMR," prefix)
static void handle_pcemr(const char *rest)
{
    if (rest[0]=='o' && rest[1]=='f' && rest[2]=='f' && rest[3]=='\0')
    {
        fsm_emergency_clear();
    }
    else
    {
        uart1_send_string("$ERR*");
    }
}
static void handle_payload(pc_reference ref[])
{
    int speed, yawrate;
    int comma = -1;
    int i;

    // All commands share a 6-character "PCxxx," prefix. Check length first so
    // the payload[5] test cannot read past the terminator.
    for (i = 0; i < 6; i++)
    {
        if (payload[i] == '\0')
        {
            uart1_send_string("$ERR*");
            return;
        }
    }

    if (payload[0] != 'P' || payload[1] != 'C' || payload[5] != ',')
    {
        uart1_send_string("$ERR*");
        return;
    }

    if (payload[2]=='R' && payload[3]=='E' && payload[4]=='F')
    {
        // Find the comma separating speed and yawrate
        for (i = 6; payload[i] != '\0'; i++)
        {
            if (payload[i] == ',')
            {
                comma = i;
                break;
            }
        }

        if (comma < 0)
        {
            uart1_send_string("$ERR*");
            return;
        }

        // Split the two numbers and parse them
        payload[comma] = '\0';

        if (parse_int(&payload[6], &speed) && parse_int(&payload[comma + 1], &yawrate) &&
            speed >= -100 && speed <= 100 && yawrate >= -100 && yawrate <= 100)
        {
            ref[0].speed = speed;
            ref[0].yawrate = yawrate;
        }
        else
        {
            uart1_send_string("$ERR*");
        }
    }
    else if (payload[2]=='L' && payload[3]=='O' && payload[4]=='G')
    {
        handle_pclog(&payload[6]);
    }
    else if (payload[2]=='C' && payload[3]=='F' && payload[4]=='G')
    {
        handle_pccfg(&payload[6]);
    }
    else if (payload[2]=='E' && payload[3]=='M' && payload[4]=='R')
    {
        handle_pcemr(&payload[6]);
    }
    else
    {
        uart1_send_string("$ERR*");
    }
}

void parser_process(pc_reference ref[])
{
    while (uart1_available())
    {
        char c = uart1_read();

        if (c == '$')
        {
            receiving = 1;
            payload_len = 0;
        }
        else if (receiving)
        {
            if (c == '*')
            {
                payload[payload_len] = '\0';
                receiving = 0;
                handle_payload(ref);
            }
            else if (payload_len < PAYLOAD_SIZE - 1)
            {
                payload[payload_len] = c;
                payload_len++;
            }
            else
            {
                // Payload too long to be valid; discard and wait for the next '$'
                receiving = 0;
                payload_len = 0;
            }
        }
    }
}
