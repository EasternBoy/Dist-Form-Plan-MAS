push!(LOAD_PATH, ".")

import Pkg
Pkg.activate(".")
# Pkg.precompile()


using LinearAlgebra, JuMP, Gurobi
using CSV, DataFrames, Dates, MAT

if !(@isdefined(GUROBI_ENV))
    const GUROBI_ENV = Gurobi.Env()
end


include("Structure.jl")
include("ADMM.jl")
include("Initiate.jl")


# Initiate 
# Central unit: number of vehicles N, number of obstacle n_obs and sampling time ts
# Vehicle: maximum sampling times
# Obstacle 


st        = 0.2 # Sample time
H         = 5   # Horizontal prediction
sim_times = 60

cen, robo, obs, Fm = Initiate3!(st)

N = cen.N

w     = Vector{opt_cl}(undef, N)
traj  = zeros(2, sim_times + 1, N)

for i in 1:N
    traj[1, 1, i] = robo[i].z[1]
    traj[2, 1, i] = robo[i].z[2]
end    

leader = 1


InitX = Matrix{Float64}(undef, H, N)
InitY = Matrix{Float64}(undef, H, N)
for i in 1:N
    InitX[:,i] = robo[i].z[1]*ones(H)
    InitY[:,i] = robo[i].z[2]*ones(H)
    w[i]       = opt_cl(InitX, InitY)
end

J = 1e9
#--------------------------#
#Network define
Neb = Vector{Vector{Int64}}(undef,N)
for i in 1:N
    if i == 1
        Neb[i] = [N,   i, i+1]
    elseif i == N
        Neb[i] = [i-1, i, 1]
    else
        Neb[i] = [i-1, i, i+1]
    end
end
#--------------------------#
PredSteps  = zeros(2, H, sim_times + 1, N)

for k in 1:sim_times
    println("")


    global w, vx, vy, J = ADMM!(cen, robo, H, Fm, leader, Neb, J)
    println("Time step $k: J = $J")

    
    for i in 1:N
        global traj[1, k+1, i]      =  w[i].Nx[1,i]
        global traj[2, k+1, i]      =  w[i].Ny[1,i]
        global PredSteps[1,:,k+1,i] =  w[i].Nx[:,i]
        global PredSteps[2,:,k+1,i] =  w[i].Ny[:,i]
        robo[i].z                   = [w[i].Nx[1,i], w[i].Ny[1,i], vx[i][1], vy[i][1]]
    end
end

display(w[1].Nx - w[2].Nx)

matwrite("traj.mat", Dict("traj" => traj))
matwrite("PredSteps.mat", Dict("PredSteps" => PredSteps))






