#!/usr/bin/env python

from cffi import FFI

ffi = FFI()

ffi.cdef("void obstacleBrushfire(int ** input, int ** output, int width, int height);")

ffi.set_source("_cpp_functions",
    """
    #include <stdio.h>
    static void obstacleBrushfire(int ** input, int ** output, int width, int height)
    {
        int i = 0;
        int j = 0;
        int step = 1;
        char changed = 1;
        while(changed == 1)
        {
            changed = 0;
            for(i = 1 ; i < width - 2 ; i = i + 1)
            {
                for(j = 1 ; j < height - 2 ; j = j + 1)
                {
                    if(output[i][j] == 0 && input[i][j] < 49 && input[i][j] != -1) // Free space
                    {
                        if(
                            output[i - 1][j] == step ||
                            output[i + 1][j] == step ||
                            output[i - 1][j - 1] == step ||
                            output[i + 1][j - 1] == step ||
                            output[i - 1][j + 1] == step ||
                            output[i + 1][j + 1] == step ||
                            output[i][j - 1] == step ||
                            output[i][j + 1] == step
                        )
                        {
                            output[i][j] = step + 1;
                            changed = 1;
                        }
                    }
                }
            }
            step = step + 1;
        }
    }
    """)
ffi.compile(verbose=False)
