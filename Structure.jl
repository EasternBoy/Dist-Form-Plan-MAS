using Gurobi, JuMP, LinearAlgebra

mutable struct opt_cl
    Nx::Matrix{Float64}
    Ny::Matrix{Float64}

    function opt_cl(Nx::Matrix{Float64}, Ny::Matrix{Float64})                 
        obj = new(Nx, Ny)
        return obj
    end
end






mutable struct central
    N::Integer # Number of agents
    n_o::Integer # Number of obstacle
    tau::Float64 # Sampling time
    init_Pos_Vehs::Matrix{Float64}  # Locations of all robots
    # dest::Matrix{Float64}  # Locations of all robots
    opti::JuMP.Model     # The solver for optimization problem

    function central(N::Integer, n_o::Integer, tau::Float64, init_Pos_Vehs::Matrix{Float64}) #dest::Matrix{Float64}
        obj = new(N, n_o, tau, init_Pos_Vehs)
        return obj
    end
end



mutable struct robot
    T::Integer     # Maximum steps
    τ::Float64   # Sampling time
    u_max::Float64 # bound control input
    v_min::Float64 
    v_max::Float64 # constant of bound constraint
    x_min::Float64 
    x_max::Float64 # constant of bound constraint
    y_min::Float64 
    y_max::Float64 # constant of bound constraint
    Rd::Float64 # Weighting matrix for a local cost function
    Ra::Float64

    pF::Vector{Float64}

    z::Vector{Float64}  # Initial position states: x, y
    u::Vector{Float64}  # Control inputs, as ODE parameters
    opti::JuMP.Model    # Distributed solver for optimization problem

    function robot(T::Integer,     tau::Float64,   u_max::Float64,      v_min::Float64,      v_max::Float64,   x_min::Float64,   x_max::Float64,
                   y_min::Float64, y_max::Float64, Rd::Float64,         Ra::Float64, x0::Vector{Float64}, pF::Vector{Float64})
        obj = new(T, tau, u_max, v_min, v_max, x_min, x_max, y_min, y_max, Rd, Ra)
        obj.z = [x0;  0.0; 0.0]    # Copy x0 initial condition with zero velocity
        obj.u = [0.0, 0.0]         # Control inputs
        obj.pF = pF
        return obj
    end
end




mutable struct obstacle
    type::Int64   #type of obstacle
    x_c::Float64 
    y_c::Float64
    L::Float64 
    W::Float64
    theta::Float64
    p1::Vector{Float64}
    p2::Vector{Float64}
    p3::Vector{Float64} 
    p4::Vector{Float64}
    # p2-------------p1
    # |   theta = 0  |
    # p3-------------p4

    function obstacle(type::Int64, x_c::Float64, y_c::Float64, L::Float64, W::Float64, theta::Float64)
        obj = new(x_c, y_c, L, W, theta)
        if type == 1 # Rectangular
            obj.x_c = x_c
            obj.y_c = y_c
            obj.L   = L
            obj.W   = W
            obj.theta = theta
            obj.p1 = [x_c + L*cos(theta) - W*sin(theta), y_c + L*sin(theta) + W*cos(theta)]
            obj.p2 = [x_c - L*cos(theta) - W*sin(theta), y_c - L*sin(theta) + W*cos(theta)]
            obj.p3 = [x_c - L*cos(theta) + W*sin(theta), y_c - L*sin(theta) - W*cos(theta)]
            obj.p4 = [x_c + L*cos(theta) + W*sin(theta), y_c + L*sin(theta) - W*cos(theta)]
        else #Ellip
            obj.x_c   = x_c
            obj.y_c   = y_c
            obj.L     = L
            obj.W     = W
            obj.theta = theta
        end
        return obj
    end
end

