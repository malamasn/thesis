#!/usr/bin/env python

from cffi import FFI

ffi = FFI()

ffi.cdef("void obstacleBrushfire(int ** input, int ** output, int width, int height);")
ffi.cdef("void gvdNeighborBrushfire(int ** brushfire, int width, int height);")
ffi.cdef("static void gvdNeighborSplitBrushfire(int ** brush_first, int ** brush_second, int width, int height);")
ffi.cdef("static void closestObstacleBrushfire(int ** brushfire, int width, int height);")
ffi.cdef("static void pointToGvdBrushfire(int ** brushfire, int width, int height);")
ffi.cdef("static void pointToPointBrushfire(int ** brushfire, int width, int height, int iterations);")
ffi.cdef("static void pointBrushfire(int ** brushfire, int width, int height);")
ffi.cdef("static void rectangularBrushfireCoverage(int ** brushfire, int width, int height, int xi, int yi, float th_deg, int cover_length, float fov, float direction);")
ffi.cdef("static void circularBrushfireCoverage(int ** brushfire, int width, int height, int xi, int yi, float th_deg, int cover_length, float fov, float direction);")
ffi.cdef("static void circularRayCastCoverage(int ** brushfire, int width, int height, int xi, int yi, float th_deg, int cover_length, float fov, float direction);")

ffi.set_source("_cpp_functions",
    """
    #include <stdio.h>
    #include <stdlib.h>
    #include <math.h>
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
    static void pointBrushfire(int ** brushfire, int width, int height)
    {
        int i = 0;
        int j = 0;
        int step = 2;
        char changed = 1;
        while(changed == 1)
        {
            changed = 0;
            for(i = 1 ; i < width - 2 ; i = i + 1)
            {
                for(j = 1 ; j < height - 2 ; j = j + 1)
                {
                    if(brushfire[i][j] == 0) // Free space
                    {
                        if(
                            brushfire[i - 1][j] == step ||
                            brushfire[i + 1][j] == step ||
                            brushfire[i - 1][j - 1] == step ||
                            brushfire[i + 1][j - 1] == step ||
                            brushfire[i - 1][j + 1] == step ||
                            brushfire[i + 1][j + 1] == step ||
                            brushfire[i][j - 1] == step ||
                            brushfire[i][j + 1] == step
                        )
                        {
                            brushfire[i][j] = step + 1;
                            changed = 1;
                        }
                    }

                }
            }
            step = step + 1;
        }
    }
    static void gvdNeighborBrushfire(int ** brushfire, int width, int height)
    {
        int i = 0;
        int j = 0;
        int step = 2;   // Start node has brushfire == 2
        char changed = 1;
        while(changed == 1)
        {
            changed = 0;
            for(i = 1 ; i < width - 2 ; i = i + 1)
            {
                for(j = 1 ; j < height - 2 ; j = j + 1)
                {
                    if(brushfire[i][j] == 1) // Gvd space not yet visited
                    {
                        if(
                            brushfire[i - 1][j] == step ||
                            brushfire[i + 1][j] == step ||
                            brushfire[i - 1][j - 1] == step ||
                            brushfire[i + 1][j - 1] == step ||
                            brushfire[i - 1][j + 1] == step ||
                            brushfire[i + 1][j + 1] == step ||
                            brushfire[i][j - 1] == step ||
                            brushfire[i][j + 1] == step
                        )
                        {
                            brushfire[i][j] = step + 1;
                            changed = 1;
                        }
                    }
                    else if(brushfire[i][j] == -1)
                    {
                        if(
                            brushfire[i - 1][j] == step ||
                            brushfire[i + 1][j] == step ||
                            brushfire[i - 1][j - 1] == step ||
                            brushfire[i + 1][j - 1] == step ||
                            brushfire[i - 1][j + 1] == step ||
                            brushfire[i + 1][j + 1] == step ||
                            brushfire[i][j - 1] == step ||
                            brushfire[i][j + 1] == step
                        )
                        {
                            brushfire[i][j] = -2;
                        }
                    }
                }
            }
            step = step + 1;
        }

    }

    static void gvdNeighborSplitBrushfire(int ** brush_first, int ** brush_second, int width, int height)
    {
        int i = 0;
        int j = 0;
        int step = 3;   // First neighbor node has brushfire == 3
        char changed = 1;
        while(changed == 1)
        {
            changed = 0;
            for(i = 1 ; i < width - 2 ; i = i + 1)
            {
                for(j = 1 ; j < height - 2 ; j = j + 1)
                {
                    if(brush_first[i][j] == 1) // Gvd space not yet visited
                    {
                        if(
                            brush_first[i - 1][j] == step ||
                            brush_first[i + 1][j] == step ||
                            brush_first[i - 1][j - 1] == step ||
                            brush_first[i + 1][j - 1] == step ||
                            brush_first[i - 1][j + 1] == step ||
                            brush_first[i + 1][j + 1] == step ||
                            brush_first[i][j - 1] == step ||
                            brush_first[i][j + 1] == step
                        )
                        {
                            brush_first[i][j] = step + 1;
                            changed = 1;
                        }
                    }
                    else if(brush_first[i][j] == -1)
                    {
                        if(
                            brush_first[i - 1][j] == step ||
                            brush_first[i + 1][j] == step ||
                            brush_first[i - 1][j - 1] == step ||
                            brush_first[i + 1][j - 1] == step ||
                            brush_first[i - 1][j + 1] == step ||
                            brush_first[i + 1][j + 1] == step ||
                            brush_first[i][j - 1] == step ||
                            brush_first[i][j + 1] == step
                        )
                        {
                            brush_first[i][j] = -2;
                        }
                    }
                }
            }
            step = step + 1;
        }

        step = 3;   // First other neighbor node has brushfire == 3
        changed = 1;
        while(changed == 1)
        {
            changed = 0;
            for(i = 1 ; i < width - 2 ; i = i + 1)
            {
                for(j = 1 ; j < height - 2 ; j = j + 1)
                {
                    if(brush_second[i][j] == 1) // Gvd space not yet visited
                    {
                        if(
                            brush_second[i - 1][j] == step ||
                            brush_second[i + 1][j] == step ||
                            brush_second[i - 1][j - 1] == step ||
                            brush_second[i + 1][j - 1] == step ||
                            brush_second[i - 1][j + 1] == step ||
                            brush_second[i + 1][j + 1] == step ||
                            brush_second[i][j - 1] == step ||
                            brush_second[i][j + 1] == step
                        )
                        {
                            brush_second[i][j] = step + 1;
                            changed = 1;
                        }
                    }
                    else if(brush_second[i][j] == -1)
                    {
                        if(
                            brush_second[i - 1][j] == step ||
                            brush_second[i + 1][j] == step ||
                            brush_second[i - 1][j - 1] == step ||
                            brush_second[i + 1][j - 1] == step ||
                            brush_second[i - 1][j + 1] == step ||
                            brush_second[i + 1][j + 1] == step ||
                            brush_second[i][j - 1] == step ||
                            brush_second[i][j + 1] == step
                        )
                        {
                            brush_second[i][j] = -2;
                        }
                    }
                }
            }
            step = step + 1;
        }
    }
    static void closestObstacleBrushfire(int ** brushfire, int width, int height)
    {
        int i = 0;
        int j = 0;
        int step = 2;
        char changed = 1;
        char found = 0;
        char last = 0;
        while(changed == 1 && last == 0)
        {
            if(found == 1)
            {
                last = 1;
            }
            changed = 0;
            for(i = 1 ; i < width - 2 ; i = i + 1)
            {
                for(j = 1 ; j < height - 2 ; j = j + 1)
                {
                    if(brushfire[i][j] == 0) // Free space
                    {
                        if(
                            brushfire[i - 1][j] == step ||
                            brushfire[i + 1][j] == step ||
                            brushfire[i - 1][j - 1] == step ||
                            brushfire[i + 1][j - 1] == step ||
                            brushfire[i - 1][j + 1] == step ||
                            brushfire[i + 1][j + 1] == step ||
                            brushfire[i][j - 1] == step ||
                            brushfire[i][j + 1] == step
                        )
                        {
                            brushfire[i][j] = step + 1;
                            changed = 1;
                        }
                    }
                    else if(brushfire[i][j] == 1)   // Obstacles
                    {
                        if(
                            brushfire[i - 1][j] == step ||
                            brushfire[i + 1][j] == step ||
                            brushfire[i - 1][j - 1] == step ||
                            brushfire[i + 1][j - 1] == step ||
                            brushfire[i - 1][j + 1] == step ||
                            brushfire[i + 1][j + 1] == step ||
                            brushfire[i][j - 1] == step ||
                            brushfire[i][j + 1] == step
                        )
                        {
                            brushfire[i][j] = -2;
                            found = 1;
                        }
                    }
                }
            }
            step = step + 1;
        }
    }
    static void pointToGvdBrushfire(int ** brushfire, int width, int height)
    {
        int i = 0;
        int j = 0;
        int step = 2;   // Start node has brushfire == 2
        char changed = 1;
        char found = 0;
        while(changed == 1 && found == 0)
        {
            changed = 0;
            for(i = 1 ; i < width - 2 ; i = i + 1)
            {
                for(j = 1 ; j < height - 2 ; j = j + 1)
                {
                    if(brushfire[i][j] == 0) // Space not yet visited
                    {
                        if(
                            brushfire[i - 1][j] == step ||
                            brushfire[i + 1][j] == step ||
                            brushfire[i - 1][j - 1] == step ||
                            brushfire[i + 1][j - 1] == step ||
                            brushfire[i - 1][j + 1] == step ||
                            brushfire[i + 1][j + 1] == step ||
                            brushfire[i][j - 1] == step ||
                            brushfire[i][j + 1] == step
                        )
                        {
                            brushfire[i][j] = step + 1;
                            changed = 1;
                        }
                    }
                    else if(brushfire[i][j] == -1)
                    {
                        if(
                            brushfire[i - 1][j] == step ||
                            brushfire[i + 1][j] == step ||
                            brushfire[i - 1][j - 1] == step ||
                            brushfire[i + 1][j - 1] == step ||
                            brushfire[i - 1][j + 1] == step ||
                            brushfire[i + 1][j + 1] == step ||
                            brushfire[i][j - 1] == step ||
                            brushfire[i][j + 1] == step
                        )
                        {
                            brushfire[i][j] = -2;
                            found = 1;
                        }
                    }
                }
            }
            step = step + 1;
        }
    }
    static void pointToPointBrushfire(int ** brushfire, int width, int height, int iterations)
    {
        int i = 0;
        int j = 0;
        int step = 2;
        char changed = 1;
        int iters_made = 0;
        while(changed == 1 && iters_made < iterations)
        {
            changed = 0;
            for(i = 1 ; i < width - 2 ; i = i + 1)
            {
                for(j = 1 ; j < height - 2 ; j = j + 1)
                {
                    if(brushfire[i][j] == 0) // Free space
                    {
                        if(
                            brushfire[i - 1][j] == step ||
                            brushfire[i + 1][j] == step ||
                            brushfire[i - 1][j - 1] == step ||
                            brushfire[i + 1][j - 1] == step ||
                            brushfire[i - 1][j + 1] == step ||
                            brushfire[i + 1][j + 1] == step ||
                            brushfire[i][j - 1] == step ||
                            brushfire[i][j + 1] == step
                        )
                        {
                            brushfire[i][j] = step + 1;
                            changed = 1;
                        }
                    }

                }
            }
            step = step + 1;
            iters_made++;
        }
    }
    static void rectangularBrushfireCoverage(int ** brushfire, int width, int height, int xi, int yi, float th_deg, int cover_length, float fov, float direction)
    {
        int i = 0;
        int j = 0;
        int xx, yy;
        double angle, theta;
        int step = 2;
        int iters_made = 0;
        while(iters_made < cover_length)
        {
            for(i = 1 ; i < width - 2 ; i = i + 1)
            {
                for(j = 1 ; j < height - 2 ; j = j + 1)
                {
                    if(brushfire[i][j] == 0) // Free space
                    {
                        if(
                            brushfire[i - 1][j] == step ||
                            brushfire[i + 1][j] == step ||
                            brushfire[i - 1][j - 1] == step ||
                            brushfire[i + 1][j - 1] == step ||
                            brushfire[i - 1][j + 1] == step ||
                            brushfire[i + 1][j + 1] == step ||
                            brushfire[i][j - 1] == step ||
                            brushfire[i][j + 1] == step
                        )
                        {
                            xx = i - xi;
                            yy = j - yi;
                            angle = atan2(yy, xx) * 180.0 / M_PI;
                            theta = angle - direction - th_deg;
                            if (theta < -180)
                            {
                                theta += 360;
                            }
                            else if (theta > 180)
                            {
                                theta -= 360;
                            }
                            if(
                                abs(theta) < fov / 2
                            )
                            {
                                brushfire[i][j] = step + 1;
                            }
                        }
                    }

                }
            }
            step = step + 1;
            iters_made++;
        }
    }
    static void circularBrushfireCoverage(int ** brushfire, int width, int height, int xi, int yi, float th_deg, int cover_length, float fov, float direction)
    {
        int i = 0;
        int j = 0;
        int xx, yy;
        double angle, theta;
        int step = 2;
        int iters_made = 0;
        while(iters_made < cover_length)
        {
            for(i = 1 ; i < width - 2 ; i = i + 1)
            {
                for(j = 1 ; j < height - 2 ; j = j + 1)
                {
                    if(brushfire[i][j] == 0) // Free space
                    {
                        if(
                            brushfire[i - 1][j] == step ||
                            brushfire[i + 1][j] == step ||
                            brushfire[i - 1][j - 1] == step ||
                            brushfire[i + 1][j - 1] == step ||
                            brushfire[i - 1][j + 1] == step ||
                            brushfire[i + 1][j + 1] == step ||
                            brushfire[i][j - 1] == step ||
                            brushfire[i][j + 1] == step
                        )
                        {
                            xx = i - xi;
                            yy = j - yi;
                            angle = atan2(yy, xx) * 180.0 / M_PI;
                            theta = angle - direction - th_deg;
                            if (theta < -180)
                            {
                                theta += 360;
                            }
                            else if (theta > 180)
                            {
                                theta -= 360;
                            }
                            if(
                                (xx * xx + yy * yy) < cover_length * cover_length &&
                                abs(theta) < fov / 2
                            )
                            {
                                brushfire[i][j] = step + 1;
                            }
                        }
                    }

                }
            }
            step = step + 1;
            iters_made++;
        }
    }
    static void circularRayCastCoverage(int ** brushfire, int width, int height, int xi, int yi, float th_deg, int cover_length, float fov, float direction)
    {
        int xx, yy;
        double angle;
        int step;
        for(double theta = -fov/2; theta <= fov/2; theta += 1)
            {
            step = 2;
            angle = (th_deg + direction + theta) *  M_PI / 180.0;

            for(int iters = 1; iters <= cover_length; iters++)
            {
                xx = (int) round(xi + cos(angle) * iters);
                yy = (int) round(yi + sin(angle) * iters);
                if (brushfire[xx][yy] == 0){
                    brushfire[xx][yy] = step;
                    step++;
                }
                else if (brushfire[xx][yy] == 1 || brushfire[xx][yy] == -1)
                {
                    break;
                }
            }
        }
    }
    """)
ffi.compile(verbose=False)
