#ifndef PARSER_H
#define PARSER_H

typedef struct {
    int speed;
    int yawrate;
} pc_reference;

void parser_process(pc_reference ref[]);

#endif
