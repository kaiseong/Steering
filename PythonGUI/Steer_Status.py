import numpy as np
class Steer_Status():
    # L2L Values
    init_target_torque = 2.40
    stuck_angle = 5
    stuck_time = 30
    gear_ratio = 263 / 72
    offset = 0

    # State
    mtrOff = 0
    mtrOn = 1
    GOREADY = 2
    Pause = 3
    Restart = 4
    L2L = 5
    ConstStr = 6
    SineStr = 7

    l2l_pass = False
    
    # Plot parameter
    time_interval = 10 / 1000  # ms
    plot_range = 10  # 10 sec data display
    
    # [xmin, xmax, ymin, ymax]    
    axis_limit = [[-5,5,-600, offset * 2 + 600],[-5,5,-6,6],[-5,5,-5,5],[-5,5,-5,5],[-5,5,-5,5]] 
    
    num = int(plot_range / time_interval)
    tor_buff = []
    ang_buff = []
    can_buff = [[],[],[]]
    serial_num = 0
    can_num = [0,0,0]
    
    # CAN time Calculate
    timestamp = [[] for _ in range(3)]
    can_diff = [10, 20]
    time_diff = 0
    
    x = np.arange(-5,5,time_interval)
    time_buff = [x,x,x,x,x]
    
    # Display Status
    disp_MS = ['OFF', 'ON'] # Motor Status
    disp_BLE = ['OFF', 'ON']
    disp_Mode = ['Set up', 'Ready', 'Sine Str Term', 'GO Ready',\
                'Pause', 'L2L', 'Const_Str', 'Sine_Str', 'L2L Term', 'Not Connected']
    
    # Status from MCU
    motor_status = 0
    BLE_status = 0
    MCU_status = -1
    
    # Const Str Status
    const_vel = 0.5
    const_range = [-50,50]
    const_cycle = 1

    # Sine Str Status
    sine_angle = 20
    sine_amp = 30
    sine_freq = 1
    sine_cycle = 1

    # Motor Parameter
    Kt = 0.95 # 0.95 Nm/A
    enc_counter = 2**14 # Encoder counter 14bit encoder 16384 = 360 degree
    motor_speed = 490 * 6 #490 rpm -> 490 * 6 deg/s (1 rpm -> 1/60 rps -> 1/60 * 360 deg/s -> 6 deg/s)
    value = [[],[]]
                                                                # Activate button When state
    able_list = [["L2L"],                                       # Set Up 
                 ["SineStr_start","ConstStr_start"],            # READY
                 ["goready", "SineStr_start"],                  # Sine Str Term
                 [],                                            # Return to Zero
                 ["ConstStr_pause","SineStr_pause", "goready"], # Pause
                 ["L2L_pause"],                                 # L2L
                 ["ConstStr_pause"],                            # Const Str
                 ["SineStr_pause"],                             # Sine Str
                 ['L2L', 'goready', "proceed"]]                            # L2L Term