from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *


class data_viewerdebug:
    def __init__(self):
        time_window_length = 100
        self.plotter = Plotter(plotting_frequency=1,  # refresh plot every 100 time steps
                               time_window=time_window_length)  # plot last time_window seconds of data
        # set up the plot window
        # define first row

        h_plots = PlotboxArgs(plots=['h_Nlp', 'h_c' , 'h_Lp'],
                              labels={'left': 'h(m)', 'bottom': 'Time (m)'},
                              time_window=time_window_length)

        # define second row
        Va_plots = PlotboxArgs(plots=['Va_Nlp', 'Va_c','Va_Lp'],
                               labels={'left': 'Va(m/s)', 'bottom': 'Time (m)'},
                               time_window=time_window_length)

        first_row = [Va_plots, h_plots]

        # define third row

        theta_plots = PlotboxArgs(plots=['theta_Nlp', 'ZeroRef','theta_Lp'],
                                  labels={'left': 'theta(deg)', 'bottom': 'Time (m)'},
                                  rad2deg=True,
                                  time_window=time_window_length)
        q_plots = PlotboxArgs(plots=['q_Nlp', 'ZeroRef','q_Lp'],
                              labels={'left': 'q(deg/s)', 'bottom': 'Time (m)'},
                              rad2deg=True,
                              time_window=time_window_length)

        second_row = [q_plots, theta_plots]

        # define fourth row

        delta_e_plots = PlotboxArgs(plots=['elevator', 'ZeroRef'],
                                    labels={'left': 'elevator(deg)', 'bottom': 'Time (m)'},
                                    rad2deg=True,
                                    time_window=time_window_length)
        delta_t_plots = PlotboxArgs(plots=['thrust', 'zeroRef'],
                                    labels={'left': 'thrust', 'bottom': 'Time (m)'},
                                    rad2deg=False,
                                    time_window=time_window_length)
        fifth_row = [delta_e_plots, delta_t_plots]
        plots = [first_row,
                 second_row,
                 fifth_row
                 ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('true_state', ['h_Nlp', 'Va_Nlp', 'theta_Nlp',
                                                        'q_Nlp', 'elevator', 'thrust'])
        self.plotter.define_input_vector('linear_estimated_state', ['h_Lp', 'Va_Lp', 'theta_Lp', 'q_Lp'])
        self.plotter.define_input_vector('commands',
                                         ['h_c', 'Va_c', 'zeroRef', 'ZeroRef', 'ZeroRef', 'zeroRef'])
        # plot timer
        self.time = 0.

    def update(self, true_state, linear_estimated_state, commanded_state, delta_input_current, ts):
        # increment time
        self.time = ts
        commands = [commanded_state.altitude_command,  # h_c
                    commanded_state.airspeed_command,  # Va_c
                    0,  # theta_c
                    0,
                    0,
                    0]

        # Add the state data in vectors
        # the order has to match the order in lines 72-76
        true_state_list = [true_state.h,
                           true_state.Va, true_state.theta,
                           true_state.q,
                           delta_input_current[1][0],
                           delta_input_current[3][0]]
        linear_estimated_state_list = [linear_estimated_state[4][0],
                                       np.power(linear_estimated_state[0][0] * linear_estimated_state[0][0] + linear_estimated_state[1][0]*linear_estimated_state[1][0],0.5),
                                       linear_estimated_state[3][0], linear_estimated_state[2][0]]
        self.plotter.add_vector_measurement('true_state', true_state_list, self.time)
        self.plotter.add_vector_measurement('linear_estimated_state', linear_estimated_state_list, self.time)
        self.plotter.add_vector_measurement('commands', commands, self.time)

        # Update and display the plot
        self.plotter.update_plots()
