push!(LOAD_PATH, "/home/hakcar/phd/ml-array-calibration/code-julia/Bayesian-Auto-Calibration/src")

using BayesianSelfCalibration

const OPT = OptimizationSettings(1e-6, 30)

function test(a)
    println(a)
end

function dyncamics_wrapper(y_vector, Ng, r)

    Na = size(r, 2)
    mimu = AccGyro(Na, Ng)

    y_inp = reshape(y_vector, :,1)
    Q_inv = eye(length(y_inp))
    w0 = y_vector[range(3*Na + 1, length = 3)]
    println("r: ", size(r))
    eta_hat = ml_mimu_dynamics(y_inp, Q_inv, w0, r, mimu, OPT)

    return eta_hat.w
end
