"Parallel version"
function ADMM!(cen::central, robo::Vector{robot}, H::Int64, Fm::Array{Float64, 3}, Leader::Int64, Neb::Vector{Vector{Int64}}, Jo::Float64; MAX_ITER = 100, thres = 1e-1)

    N = cen.N

    ξ     = Vector{opt_cl}(undef,N)
    w     = Vector{opt_cl}(undef,N)
    λ     = Matrix{opt_cl}(undef,N,N)
    vx    = Vector{Vector}(undef,N)
    vy    = Vector{Vector}(undef,N)

    # Initial position setup for all vehicles
    InitX = Matrix{Float64}(undef, H, N)
    InitY = Matrix{Float64}(undef, H, N)
    for i in 1:N
        InitX[:,i] = robo[i].z[1]*ones(H)
        InitY[:,i] = robo[i].z[2]*ones(H)
    end


    for i in 1:N
        w[i] = opt_cl(InitX, InitY)
        for j in 1:N
            λ[i,j] = opt_cl(zeros(H,N), zeros(H,N))
        end
    end


    Jn = zeros(N)

    t0 = time_ns()
    for n in 1:MAX_ITER
        #println("Iterative step $l")
        # Parallel manner in each vehicle
        ρ = 1.1^n

        for i in 1:N
            ξ[i] = Local_Update!(robo[i], cen, i, H, w, λ[i,:], Fm, Leader, Neb, ρ, Jo)
        end
 
        for i in 1:N
            w[i], vx[i], vy[i], Jn[i] = Constraint_Update!(robo[i], cen, i, H, w, ξ, λ[:,i], Fm, Leader, Neb, ρ, Jo)
        end

        # Update dual variable
        for i in 1:N
            for j in Neb[i]
                λ[i,j].Nx = λ[i,j].Nx + ρ*(ξ[i].Nx - w[j].Nx)
                λ[i,j].Ny = λ[i,j].Ny + ρ*(ξ[i].Ny - w[j].Ny)
            end
        end

        ter = 0
        
        for i in 1:N
            ter = ter + norm(vec(ξ[i].Nx - w[i].Nx))
            ter = ter + norm(vec(ξ[i].Ny - w[i].Ny))
        end

        ter = round(ter, digits = 4)
        # println(ter)
        
        if ter <= thres
            break
        end
    end

    t1 = time_ns()
    dt = (t1-t0)/1e9
    dt = round(dt/N; digits = 3)
    println("Average time for each: $dt (s)")
    println("")

    return w, vx, vy, Jn[Leader]
end



function Local_Update!(robo::robot, cen::central, ii::Int64, H::Int64, w::Vector{opt_cl}, λ::Vector{opt_cl}, Fm::Array{Float64, 3}, ℓ::Int64, Neb::Vector{Vector{Int64}}, ρ::Float64, Jo::Float64)
    # Find pairs of robots that need collision avoidance
    N   = cen.N

    robo.opti = JuMP.Model(() -> Gurobi.Optimizer(GUROBI_ENV))
    set_silent(robo.opti)

    xF = robo.pF[1]
    yF = robo.pF[2]

    # Variables
    Nx = @variable(robo.opti, [1:H, 1:N]) # slack variable
    Ny = @variable(robo.opti, [1:H, 1:N]) # slack variable
    
    # @variable(robo.opti, Jn)

    # Objective function
    α  = 0.5
    J  = 0

    if ii == ℓ
        J = J + sum((Nx[h,ℓ] - xF)^2 + (Ny[h,ℓ] - yF)^2 for h in 1:H)
        for i in N:-1:1
            for j in i-1:-1:1      
                J = J + α*sum((Nx[h,i] - Nx[h,j] - Fm[i,j,1])^2 + (Ny[h,i] - Ny[h,j] - Fm[i,j,2])^2 for h in 1:H)
            end
        end
    end

    for j in Neb[ii]
        J = J + ρ/2*dot(vec(Nx - w[j].Nx + λ[j].Nx/ρ), vec(Nx - w[j].Nx + λ[j].Nx/ρ)) + ρ/2*dot(vec(Ny - w[j].Ny + λ[j].Ny/ρ), vec(Ny - w[j].Ny + λ[j].Ny/ρ))
    end

    @objective(robo.opti, MOI.MIN_SENSE, J)
    status = JuMP.optimize!(robo.opti)

    return opt_cl(JuMP.value.(Nx),  JuMP.value.(Ny))
end





function Constraint_Update!(robo::robot, cen::central, ii::Int64, H::Int64, w::Vector{opt_cl}, ξ::Vector{opt_cl}, λ::Vector{opt_cl}, Fm::Array{Float64, 3}, ℓ::Int64, Neb::Vector{Vector{Int64}}, ρ::Float64, Jo::Float64)
    # Find pairs of robots that need collision avoidance
    N   = cen.N

    robo.opti = JuMP.Model(() -> Gurobi.Optimizer(GUROBI_ENV))
    set_silent(robo.opti)

    xF = robo.pF[1]
    yF = robo.pF[2]

    # Variables
    ux = @variable(robo.opti, [1:H])
    uy = @variable(robo.opti, [1:H])
    vx = @variable(robo.opti, [1:H])
    vy = @variable(robo.opti, [1:H])
    Nx = @variable(robo.opti, [1:H, 1:N]) # slack variable
    Ny = @variable(robo.opti, [1:H, 1:N]) # slack variable
    
    # @variable(robo.opti, Jn)

    # Objective function
    α  = 0.5
    J  = 0
    Jw = 0;

    if ii == ℓ
        J = J + sum((Nx[h,ℓ] - xF)^2 + (Ny[h,ℓ] - yF)^2 for h in 1:H)
        for i in N:-1:1
            for j in i-1:-1:1      
                for h in 1:H
                    J = J + α*(Nx[h,i] - Nx[h,j] - Fm[i,j,1])^2 + α*(Ny[h,i] - Ny[h,j] - Fm[i,j,2])^2 
                end
            end
        end
        # @constraint(robo.opti, J  <= Jo)
    end

    for j in Neb[ii]
        Jw = Jw + ρ/2*dot(vec(ξ[j].Nx - Nx + λ[j].Nx/ρ), vec(ξ[j].Nx - Nx + λ[j].Nx/ρ)) + ρ/2*dot(vec(ξ[j].Ny - Ny + λ[j].Ny/ρ), vec(ξ[j].Ny - Ny + λ[j].Ny/ρ))
    end
    @objective(robo.opti, MOI.MIN_SENSE, Jw)





    # Constraints
    @constraints(robo.opti, begin
           -robo.u_max .<= ux .<= robo.u_max
           -robo.u_max .<= uy .<= robo.u_max
            robo.v_min .<= vx .<= robo.v_max
            robo.v_min .<= vy .<= robo.v_max
        end)
    
    for i in 1:N
        for h in 1:H
            @constraint(robo.opti, robo.x_min + robo.Ra .<= Nx[h,i] .<= robo.x_max - robo.Ra)
            @constraint(robo.opti, robo.y_min + robo.Ra .<= Ny[h,i] .<= robo.y_max - robo.Ra)
        end
    end

    # Dynamic Constraints
    for h in 1:H
        if h == 1
            @constraint(robo.opti, Nx[h,ii] - robo.z[1]  - robo.τ*robo.z[3] - (robo.τ)^2*ux[h] == 0)
            @constraint(robo.opti, Ny[h,ii] - robo.z[2]  - robo.τ*robo.z[4] - (robo.τ)^2*uy[h] == 0)
            @constraint(robo.opti, vx[h]    - robo.z[3]  - robo.τ*ux[h]                        == 0)
            @constraint(robo.opti, vy[h]    - robo.z[4]  - robo.τ*uy[h]                        == 0)
        else
            @constraint(robo.opti, Nx[h,ii] - Nx[h-1,ii] - robo.τ*vx[h-1] - (robo.τ)^2*ux[h] == 0)
            @constraint(robo.opti, Ny[h,ii] - Ny[h-1,ii] - robo.τ*vy[h-1] - (robo.τ)^2*uy[h] == 0)
            @constraint(robo.opti, vx[h]    - vx[h-1]    - robo.τ*ux[h]                      == 0)
            @constraint(robo.opti, vy[h]    - vy[h-1]    - robo.τ*uy[h]                      == 0)  
        end
    end
    
    ϵ = 0.05

    # Collision avoidance
    for j in 1:N
        if j != ii
            for h in 1:H
                dis = sqrt((w[ii].Nx[h,ii] - w[ii].Nx[h,j])^2 + (w[ii].Ny[h,ii] - w[ii].Ny[h,j])^2)
                ex =       (w[ii].Nx[h,ii] - w[ii].Nx[h,j])/dis
                ey =       (w[ii].Ny[h,ii] - w[ii].Ny[h,j])/dis
                @constraint(robo.opti, (Nx[h,ii] - Nx[h,j])*ex + (Ny[h,ii] - Ny[h,j])*ey >= 2*robo.Ra + ϵ)
            end
        end
    end


    # Now solve and get results
    status = JuMP.optimize!(robo.opti)

    Jn = 0
    if ii == ℓ
        Jn = Jn + sum((JuMP.value.(Nx)[h,ℓ] - xF)^2 + (JuMP.value.(Ny)[h,ℓ] - yF)^2 for h in 1:H)
        for i in N:-1:1
            for j in i-1:-1:1      
                for h in 1:H
                    Jn = Jn + α*(JuMP.value.(Nx)[h,i] - JuMP.value.(Nx)[h,j] - Fm[i,j,1])^2 + α*(JuMP.value.(Ny)[h,i] - JuMP.value.(Ny)[h,j] - Fm[i,j,2])^2 
                end
            end
        end
    end

    return opt_cl(JuMP.value.(Nx),  JuMP.value.(Ny)), JuMP.value.(vx), JuMP.value.(vy), Jn
end
