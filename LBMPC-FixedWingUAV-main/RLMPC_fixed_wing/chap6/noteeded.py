"""while sim_time < SIM.end_time:
    print(v)

    v += 1
    # -------controller-------------


    previous_states = deepcopy(mav.msg_true_state)

    sim_time += SIM.ts_simulation
    mav.update_state(delta2, current_wind)
    x_longitudinal_nlp = np.array([
        [previous_states.ur],  # (3)
        [previous_states.wr],  # (5)
        [previous_states.q],  # (10)
        [previous_states.theta],  # (11)
        [previous_states.h]])
    # pdb.set_trace()
    x_linear_deviation = x_longitudinal_nlp - long_trim_state
    # x_linear_deviation = x_linear_prediction - long_trim_state
    x_linear_prediction = next_step(x_linear_deviation, np.array([
        [delta2[1][0]],  # (3)
        [delta2[3][0]],  # (5)
        [delta2[4][0]]
    ])) + long_trim_state
    datas_view.update(mav.msg_true_state,  # true states
                      x_linear_prediction,  # linear_estimated states
                      commands,  # commanded states
                      delta2,
                      (timer() - start) / 60)
return"""