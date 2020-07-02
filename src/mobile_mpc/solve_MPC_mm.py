#!/usr/bin/env python

import os
import sys

sys.path.append(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "../../forcesLib/mobileManipulator/"
    )
)

import time
import numpy as np
import mm_MPC_py

def blockPrint():
    sys.stdout = open(os.devnull, 'w')

def enablePrint():
    sys.stdout = sys.__stdout__

def solve_MPC_mm(params):
    blockPrint()
    res = mm_MPC_py.mm_MPC_solve(params)
    enablePrint()
    return res
