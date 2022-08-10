#ifndef UTILS_H
#define UTILS_H

extern int shouldStart;
extern int shouldRunOffset;

float mapf(float input, float inFrom, float inTo, float outFrom, float outTo);
void setStart(int val);
void setOffset(int val);

#endif