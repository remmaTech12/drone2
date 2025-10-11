flowchart LR
  subgraph Inputs[Inputs / Sensors]
    VXREF[vx_ref] 
    VYREF[vy_ref] 
    ZREF[z_ref] 
    PSIREF[ψ_ref]
    DX[dx] 
    DY[dy] 
    Z[z] 
    WX[wx] 
    WY[wy] 
    WZ[wz]
  end

  subgraph Estimation[Estimation]
    DZEST[dz estimate]
    ATT[Attitude Estimator: q, R, ψ]
  end

  subgraph Outer[Outer loops 50–100 Hz]
    VelCtrl[Horizontal Velocity Ctrl → a_xy_ref]
    AltCtrl[Altitude Ctrl PID → T]
  end

  subgraph Middle[Attitude loop 200–500 Hz]
    AttCtrl[Attitude PI → ω_ref]
  end

  subgraph Inner[Rate loop 1–4 kHz]
    RateCtrl[Rate PID (D: LPF on ω)]
  end

  subgraph Alloc[Mixer / Motors]
    Mixer[Mixer: [T, τx, τy, τz] → f_i]
    ESC[ESC / Motors]
  end

  %% connections
  VXREF --> VelCtrl
  VYREF --> VelCtrl
  DX --> VelCtrl
  DY --> VelCtrl

  ZREF --> AltCtrl
  Z --> AltCtrl
  DZEST --> AltCtrl

  ATT --> AttCtrl
  PSIREF --> AttCtrl

  VelCtrl --> Mixer
  AltCtrl --> Mixer
  AttCtrl --> RateCtrl
  RateCtrl --> Mixer
  Mixer --> ESC
