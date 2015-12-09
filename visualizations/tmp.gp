set xrange [-400.0: 400.0] 
        set yrange [-400.0: 400.0] 
        set pm3d 
        set view map 
        unset key 
        set size square 
        unset arrow 
        set arrow from 0, 0 to -150, 0 nohead front lt 3 
        set arrow from -150, 0 to -150, -50 nohead front lt 3 
        set arrow from -150, -50 to 0, -50 nohead front lt 3 
        set arrow from 0, -50 to 0, 0 nohead front lt 3 
        set arrow from 200, 100 to 200, 330 nohead front lt 3 
        set arrow from 200, 330 to 300, 330 nohead front lt 3 
        set arrow from 300, 330 to 300, 100 nohead front lt 3 
        set arrow from 300, 100 to 200, 100 nohead front lt 3 
        set palette model RGB functions 1-gray, 1-gray, 1-gray 
        set isosamples 100 
        mu_x = --226.697753 
        mu_y = -2.808109 
        sigma_x = 10.560324 
        sigma_y = 10.560324 
        rho = 0.000000 
        splot 1.0/(2.0 * pi * sigma_x * sigma_y * sqrt(1 - rho**2) )        * exp(-1.0/2.0 * ((x+mu_x)**2 / sigma_x**2 + (y+mu_y)**2 / sigma_y**2        - 2.0*rho*(x + mu_x)*(y + mu_y)/(sigma_x*sigma_y) ) ) with pm3d
        set term png
        set output "visualizations/kalman_1449693461.082712.png"
        replot