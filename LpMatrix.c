/***********************************************************************
** Copyright (C) 2019 LP-Research
** All rights reserved.
** Contact: LP-Research (info@lp-research.com)
***********************************************************************/

#include "LpMatrix.h"

void createIdentity3x3(LpMatrix3x3f* dest)
{
    matZero3x3(dest);

    dest->data[0][0] = 1;
    dest->data[1][1] = 1;
    dest->data[2][2] = 1;
}

void createIdentity4x4(LpMatrix4x4f* dest)
{
    matZero4x4(dest);

    dest->data[0][0] = 1;
    dest->data[1][1] = 1;
    dest->data[2][2] = 1;
    dest->data[3][3] = 1;
}

void matZero3x3(LpMatrix3x3f* dest)
{
    dest->data[0][0] = 0;
    dest->data[0][1] = 0;
    dest->data[0][2] = 0;

    dest->data[1][0] = 0;
    dest->data[1][1] = 0;
    dest->data[1][2] = 0;

    dest->data[2][0] = 0;
    dest->data[2][1] = 0;
    dest->data[2][2] = 0;
}

void matZero3x4(LpMatrix3x4f* dest)
{
    dest->data[0][0] = 0;
    dest->data[0][1] = 0;
    dest->data[0][2] = 0;
    dest->data[0][3] = 0;

    dest->data[1][0] = 0;
    dest->data[1][1] = 0;
    dest->data[1][2] = 0;
    dest->data[1][3] = 0;

    dest->data[2][0] = 0;
    dest->data[2][1] = 0;
    dest->data[2][2] = 0;
    dest->data[2][3] = 0;
}

void matZero4x3(LpMatrix4x3f* dest)
{
    dest->data[0][0] = 0;
    dest->data[0][1] = 0;
    dest->data[0][2] = 0;

    dest->data[1][0] = 0;
    dest->data[1][1] = 0;
    dest->data[1][2] = 0;

    dest->data[2][0] = 0;
    dest->data[2][1] = 0;
    dest->data[2][2] = 0;

    dest->data[3][0] = 0;
    dest->data[3][1] = 0;
    dest->data[3][2] = 0;
}

void matZero4x4(LpMatrix4x4f* dest)
{
    dest->data[0][0] = 0;
    dest->data[0][1] = 0;
    dest->data[0][2] = 0;
    dest->data[0][3] = 0;

    dest->data[1][0] = 0;
    dest->data[1][1] = 0;
    dest->data[1][2] = 0;
    dest->data[1][3] = 0;

    dest->data[2][0] = 0;
    dest->data[2][1] = 0;
    dest->data[2][2] = 0;
    dest->data[2][3] = 0;

    dest->data[3][0] = 0;
    dest->data[3][1] = 0;
    dest->data[3][2] = 0;
    dest->data[3][3] = 0;
}

void vectZero3x1(LpVector3f* dest)
{
    dest->data[0] = 0;
    dest->data[1] = 0;
    dest->data[2] = 0;
}

void vectZero4x1(LpVector4f* dest)
{
    dest->data[0] = 0;
    dest->data[1] = 0;
    dest->data[2] = 0;
    dest->data[3] = 0;
}

#ifdef __WIN32

#include "stdio.h"

void print4x4(LpMatrix4x4f m)
{
    int i, j;

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            printf("%f, ", m.data[i][j]);
        }
        printf("\n");
    }
}

#endif