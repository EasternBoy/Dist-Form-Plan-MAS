using JuMP, Gurobi
import OSQP


function Initiate1!(ts::Float64)

    n_veh = 2
    n_obs = 3

    init_pos_robo = [ 1.   3.
                    ; 5.   3.
                    ]

    dest = [  1.  3.
            ; 7.  9.
            ]

    T = 20*ones(Int64,n_veh)

    Fm = zeros(n_veh, n_veh, 2)

        
    for i = 1:n_veh
        for j = 1:n_veh
            Fm[i,j,1] = (init_pos_robo[i,1] - init_pos_robo[j,1])
            Fm[i,j,2] = (init_pos_robo[i,2] - init_pos_robo[j,2])
        end
    end


    init_pos_robo = [ 5.   3.
                    ; 1.   3.
                    ]


    cen  = central(n_veh, n_obs, ts, init_pos_robo)
    robo = Vector{robot}(undef, n_veh)
    obs  = Vector{obstacle}(undef, n_obs)


    u_max = 10.
    v_min =-2.0
    v_max = 2.0
    x_min = 0.5
    x_max = 15.0
    y_min = 0.5
    y_max = 15.0
    Rd    = 2.0
    Ra    = 0.3


    matwrite("init_pos.mat", Dict("init_pos_robo" => init_pos_robo))


    solver = Gurobi.Optimizer

    for i in 1:n_veh
        robo[i] = robot(T[i], ts, u_max, v_min, v_max, x_min, x_max, y_min, y_max, Rd, Ra, init_pos_robo[i,:], dest[i,:])
        # robo[i].opti = JuMP.Model(JuMP.with_optimizer(solver, GUROBI_ENV))
        # set_silent(robo[i].opti)
    end



    obs[1]  = obstacle(1, 4.,   7.,   5.,  1., π/2) #Rectangular
    obs[2]  = obstacle(1, 10.0, 11.,  3.,  1., 0.)  #Rectangular
    obs[3]  = obstacle(1, 11.0, 4.,   4.,  1., 0.)  #Rectangular

    return cen, robo, obs, Fm

end



function Initiate2!(ts::Float64)

    n_veh = 4
    n_obs = 3

    init_pos_robo = [ 1.   1.
                    ; 1.   2.
                    ; 2.   1.
                    ; 2.   2.
                    ]

    dest = [  7.  7.
            ; 7.  9.
            ; 9.  7.
            ; 9.  9.
            ]

    T = 20*ones(Int64,n_veh)

    Fm = zeros(n_veh, n_veh, 2)

        
    for i = 1:n_veh
        for j = 1:n_veh
            Fm[i,j,1] = (init_pos_robo[i,1] - init_pos_robo[j,1])*2
            Fm[i,j,2] = (init_pos_robo[i,2] - init_pos_robo[j,2])*2
        end
    end


    init_pos_robo = [ 2.   2.
                    ; 2.   1.
                    ; 1.   2.
                    ; 1.   1.
                    ]


    cen  = central(n_veh, n_obs, ts, init_pos_robo)
    robo = Vector{robot}(undef, n_veh)
    obs  = Vector{obstacle}(undef, n_obs)


    u_max = 10.
    v_min =-1.5
    v_max = 1.5
    x_min = 0.5
    x_max = 15.0
    y_min = 0.5
    y_max = 15.0
    Rd    = 2.0
    Ra    = 0.3


    matwrite("init_pos.mat", Dict("init_pos_robo" => init_pos_robo))


    solver = Gurobi.Optimizer

    for i in 1:n_veh
        robo[i] = robot(T[i], ts, u_max, v_min, v_max, x_min, x_max, y_min, y_max, Rd, Ra, init_pos_robo[i,:], dest[i,:])
        # robo[i].opti = JuMP.Model(JuMP.with_optimizer(solver, GUROBI_ENV))
        # set_silent(robo[i].opti)
    end



    obs[1]  = obstacle(1, 4.,   7.,   5.,  1., π/2) #Rectangular
    obs[2]  = obstacle(1, 10.0, 11.,  3.,  1., 0.)  #Rectangular
    obs[3]  = obstacle(1, 11.0, 4.,   4.,  1., 0.)  #Rectangular

    return cen, robo, obs, Fm

end




function Initiate3!(ts::Float64)

    n_veh = 9
    n_obs = 3

    init_pos_robo = [ 1.   1.
                    ; 3.   1.
                    ; 5.   1.
                    ; 1.   3.
                    ; 3.   3.
                    ; 5.   3.
                    ; 1.   5.
                    ; 3.   5.
                    ; 5.   5.
                    ]

    dest = [  8.  8.
            ; 8.  8.
            ; 8.  8.
            ; 8.  8.
            ; 8.  8.
            ; 8.  8.
            ; 8.  8.
            ; 8.  8.
            ; 8.  8.
            ]

    T = 20*ones(Int64,n_veh)
    
    matwrite("dest.mat", Dict("dest" => dest))
    matwrite("init_pos.mat", Dict("init_pos_robo" => init_pos_robo))

    Fm = zeros(n_veh, n_veh, 2)

        
    for i = 1:n_veh
        for j = 1:n_veh
            Fm[i,j,1] = (init_pos_robo[i,1] - init_pos_robo[j,1])/2
            Fm[i,j,2] = (init_pos_robo[i,2] - init_pos_robo[j,2])/2
        end
    end


    init_pos_robo = [ 3.   5.
                    ; 1.   1.
                    ; 1.   3.
                    ; 5.   1.
                    ; 5.   3.
                    ; 3.   1.
                    ; 1.   5.
                    ; 3.   3.
                    ; 5.   5.
                    ]


    cen  = central(n_veh, n_obs, ts, init_pos_robo)
    robo = Vector{robot}(undef, n_veh)
    obs  = Vector{obstacle}(undef, n_obs)


    u_max = 10.
    v_min =-2.
    v_max = 2.
    x_min = 0.5
    x_max = 15.0
    y_min = 0.5
    y_max = 15.0
    Rd    = 2.0
    Ra    = 0.3

    solver = Gurobi.Optimizer

    for i in 1:n_veh
        robo[i] = robot(T[i], ts, u_max, v_min, v_max, x_min, x_max, y_min, y_max, Rd, Ra, init_pos_robo[i,:], dest[i,:])
        # robo[i].opti = JuMP.Model(JuMP.with_optimizer(solver, GUROBI_ENV))
        # set_silent(robo[i].opti)
    end



    obs[1]  = obstacle(1, 4.,   7.,   5.,  1., π/2) #Rectangular
    obs[2]  = obstacle(1, 10.0, 11.,  3.,  1., 0.)  #Rectangular
    obs[3]  = obstacle(1, 11.0, 4.,   4.,  1., 0.)  #Rectangular

    return cen, robo, obs, Fm

end




function Initiate3!(ts::Float64)

    n_veh = 9
    n_obs = 3

    de_pos =[ 2.   8.
            ; 1.   8.
            ; 3.   8.
            ; 1.5  9.
            ; 2.5  9.
            ; 1.5  7.
            ; 2.5  7.
            ; 2.   6.
            ; 2.   10.
            ]

    dest = [2.,  8.]

    T = 20*ones(Int64,n_veh)
    
    Fm = zeros(n_veh, n_veh, 2)

        
    for i = 1:n_veh
        for j = 1:n_veh
            Fm[i,j,1] = de_pos[i,1] - de_pos[j,1]
            Fm[i,j,2] = de_pos[i,2] - de_pos[j,2]
        end
    end


    init_pos_robo = [ 3.   5.
                    ; 1.   1.
                    ; 1.   3.
                    ; 5.   1.
                    ; 5.   3.
                    ; 3.   1.
                    ; 1.   5.
                    ; 3.   3.
                    ; 5.   5.
                    ]


    cen  = central(n_veh, n_obs, ts, init_pos_robo)
    robo = Vector{robot}(undef, n_veh)
    obs  = Vector{obstacle}(undef, n_obs)


    u_max = 10.
    v_min =-2.
    v_max = 2.
    x_min = 0.5
    x_max = 15.0
    y_min = 0.5
    y_max = 15.0
    Rd    = 2.0
    Ra    = 0.3

    solver = Gurobi.Optimizer

    for i in 1:n_veh
        robo[i] = robot(T[i], ts, u_max, v_min, v_max, x_min, x_max, y_min, y_max, Rd, Ra, init_pos_robo[i,:], dest)
        # robo[i].opti = JuMP.Model(JuMP.with_optimizer(solver, GUROBI_ENV))
        # set_silent(robo[i].opti)
    end



    obs[1]  = obstacle(1, 4.,   7.,   5.,  1., π/2) #Rectangular
    obs[2]  = obstacle(1, 10.0, 11.,  3.,  1., 0.)  #Rectangular
    obs[3]  = obstacle(1, 11.0, 4.,   4.,  1., 0.)  #Rectangular

    return cen, robo, obs, Fm

end



function Initiate4!(ts::Float64)

    n_veh = 9
    n_obs = 3

    de_pos =[ 9.   9.
            ; 10.  8.
            ; 9.   8.
            ; 7.   9.
            ; 8.   8.
            ; 9.   7.
            ; 7.   8.
            ; 7.   7.
            ; 6.   8.
            ]

    dest = [  9.,  9.]

    T = 20*ones(Int64,n_veh)
    
    Fm = zeros(n_veh, n_veh, 2)

        
    for i in n_veh:-1:1
        for j in i-1:-1:1
            Fm[i,j,1] = de_pos[i,1] - de_pos[j,1]
            Fm[i,j,2] = de_pos[i,2] - de_pos[j,2]
        end
    end


    init_pos_robo = [ 2.   8.
                    ; 1.   8.
                    ; 3.   8.
                    ; 1.5  9.
                    ; 2.5  9.
                    ; 1.5  7.
                    ; 2.5  7.
                    ; 2.   6.
                    ; 2.   10.
                    ]


    cen  = central(n_veh, n_obs, ts, init_pos_robo)
    robo = Vector{robot}(undef, n_veh)
    obs  = Vector{obstacle}(undef, n_obs)


    u_max = 10.
    v_min =-1.5
    v_max = 1.5
    x_min = 0.5
    x_max = 15.0
    y_min = 0.5
    y_max = 15.0
    Rd    = 2.0
    Ra    = 0.3

    solver = Gurobi.Optimizer

    for i in 1:n_veh
        robo[i] = robot(T[i], ts, u_max, v_min, v_max, x_min, x_max, y_min, y_max, Rd, Ra, init_pos_robo[i,:], dest)
        # robo[i].opti = JuMP.Model(JuMP.with_optimizer(solver, GUROBI_ENV))
        # set_silent(robo[i].opti)
    end



    obs[1]  = obstacle(1, 4.,   7.,   5.,  1., π/2) #Rectangular
    obs[2]  = obstacle(1, 10.0, 11.,  3.,  1., 0.)  #Rectangular
    obs[3]  = obstacle(1, 11.0, 4.,   4.,  1., 0.)  #Rectangular

    return cen, robo, obs, Fm

end



function Initiate5!(ts::Float64)

    n_veh = 9
    n_obs = 3

    de_pos =[ 9.   2.
            ; 9.   1.
            ; 9.   3.
            ; 8.   1.
            ; 8.   2.
            ; 8.   3.
            ; 10.  3.
            ; 10.  2.
            ; 10.  1.
            ]

    dest = [9.,   2.]

    T = 20
    
    Fm = zeros(n_veh, n_veh, 2)

        
    for i in n_veh:-1:1
        for j in i-1:-1:1
            Fm[i,j,1] = de_pos[i,1] - de_pos[j,1]
            Fm[i,j,2] = de_pos[i,2] - de_pos[j,2]
        end
    end


    init_pos_robo = [ 9.   9.
                    ; 10.  8.
                    ; 9.   8.
                    ; 7.   9.
                    ; 8.   8.
                    ; 9.   7.
                    ; 7.   8.
                    ; 7.   7.
                    ; 6.   8.
                    ]


    cen  = central(n_veh, n_obs, ts, init_pos_robo)
    robo = Vector{robot}(undef, n_veh)
    obs  = Vector{obstacle}(undef, n_obs)


    u_max = 10.
    v_min =-1.5
    v_max = 1.5
    x_min = 0.5
    x_max = 15.0
    y_min = 0.5
    y_max = 15.0
    Rd    = 2.0
    Ra    = 0.3

    solver = Gurobi.Optimizer

    for i in 1:n_veh
        robo[i] = robot(T, ts, u_max, v_min, v_max, x_min, x_max, y_min, y_max, Rd, Ra, init_pos_robo[i,:], dest)
        # robo[i].opti = JuMP.Model(JuMP.with_optimizer(solver, GUROBI_ENV))
        # set_silent(robo[i].opti)
    end



    obs[1]  = obstacle(1, 4.,   7.,   5.,  1., π/2) #Rectangular
    obs[2]  = obstacle(1, 10.0, 11.,  3.,  1., 0.)  #Rectangular
    obs[3]  = obstacle(1, 11.0, 4.,   4.,  1., 0.)  #Rectangular

    return cen, robo, obs, Fm

end