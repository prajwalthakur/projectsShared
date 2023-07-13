from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *


class data_viewer:
    def __init__(self):
        time_window_length = 100
        self.plotter = Plotter(plotting_frequency=20,  # refresh plot every 100 time steps
                               time_window=time_window_length)  # plot last time_window seconds of data
        # set up the plot window
        # define first row
        pn_plots = PlotboxArgs(plots=['pn', 'pn_e'],
                               labels={'left': 'pn(m)', 'bottom': 'Time (m)'},
                               time_window=time_window_length)
        pe_plots = PlotboxArgs(plots=['pe', 'pe_e'],
                               labels={'left': 'pe(m)', 'bottom': 'Time (m)'},
                               time_window=time_window_length)
        h_plots = PlotboxArgs(plots=['h', 'h_e', 'h_c'],
                              labels={'left': 'h(m)', 'bottom': 'Time (m)'},
                              time_window=time_window_length)
        wind_plots = PlotboxArgs(plots=['wn', 'wn_e', 'we', 'we_e'],
                                 labels={'left': 'wind(m/s)', 'bottom': 'Time (m)'},
                                 time_window=time_window_length)
        first_row = [pn_plots, pe_plots, h_plots, wind_plots]

        # define second row
        Va_plots = PlotboxArgs(plots=['Va', 'Va_e', 'Va_c'],
                               labels={'left': 'Va(m/s)', 'bottom': 'Time (m)'},
                               time_window=time_window_length)
        alpha_plots = PlotboxArgs(plots=['alpha', 'alpha_e'],
                                  labels={'left': 'alpha(deg)', 'bottom': 'Time (m)'},
                                  rad2deg=True,
                                  time_window=time_window_length)
        beta_plots = PlotboxArgs(plots=['beta', 'beta_e'],
                                 labels={'left': 'beta(deg)', 'bottom': 'Time (m)'},
                                 rad2deg=True,
                                 time_window=time_window_length)
        Vg_plots = PlotboxArgs(plots=['Vg', 'Vg_e'],
                               labels={'left': 'Vg(m/s)', 'bottom': 'Time (m)'},
                               time_window=time_window_length)
        second_row = [Va_plots, alpha_plots, beta_plots, Vg_plots]

        # define third row
        phi_plots = PlotboxArgs(plots=['phi', 'phi_e', 'phi_c'],
                                labels={'left': 'phi(deg)', 'bottom': 'Time (m)'},
                                rad2deg=True,
                                time_window=time_window_length)
        theta_plots = PlotboxArgs(plots=['theta', 'theta_e', 'theta_c'],
                                  labels={'left': 'theta(deg)', 'bottom': 'Time (m)'},
                                  rad2deg=True,
                                  time_window=time_window_length)
        psi_plots = PlotboxArgs(plots=['psi', 'psi_e'],
                                labels={'left': 'psi(deg)', 'bottom': 'Time (m)'},
                                rad2deg=True,
                                time_window=time_window_length)
        chi_plots = PlotboxArgs(plots=['chi', 'chi_e', 'chi_c'],
                                labels={'left': 'chi(deg)', 'bottom': 'Time (m)'},
                                rad2deg=True,
                                time_window=time_window_length)
        third_row = [phi_plots, theta_plots, psi_plots, chi_plots]

        # define fourth row
        p_plots = PlotboxArgs(plots=['p', 'p_e'],
                              labels={'left': 'p(deg/s)', 'bottom': 'Time (m)'},
                              rad2deg=True,
                              time_window=time_window_length)
        q_plots = PlotboxArgs(plots=['q', 'q_c'],
                              labels={'left': 'q(deg/s)', 'bottom': 'Time (m)'},
                              rad2deg=True,
                              time_window=time_window_length)
        r_plots = PlotboxArgs(plots=['r', 'r_e'],
                              labels={'left': 'r(deg)', 'bottom': 'Time (m)'},
                              rad2deg=True,
                              time_window=time_window_length)
        gyro_plots = PlotboxArgs(plots=['bx', 'bx_e', 'by', 'by_e', 'bz', 'bz_e'],
                                 labels={'left': 'bias(deg/s)', 'bottom': 'Time (m)'},
                                 rad2deg=True,
                                 time_window=time_window_length)
        fourth_row = [p_plots, q_plots, r_plots, gyro_plots]
        delta_a_plots = PlotboxArgs(plots=['aileron'],
                                    labels={'left': 'aileron(deg)', 'bottom': 'Time (m)'},
                                    time_window=time_window_length,
                                    rad2deg=True)

        delta_r_plots = PlotboxArgs(plots=['rudder'],
                                    labels={'left': 'rudder(deg)', 'bottom': 'Time (m)'},
                                    rad2deg=True,
                                    time_window=time_window_length)
        delta_e_plots = PlotboxArgs(plots=['elevator','e_c'],
                                    labels={'left': 'elevator(deg)', 'bottom': 'Time (m)'},
                                    rad2deg=True,
                                    time_window=time_window_length)
        delta_t_plots = PlotboxArgs(plots=['thrust','t_c'],
                                    labels={'left': 'thrust', 'bottom': 'Time (m)'},
                                    rad2deg=False,
                                    time_window=time_window_length)
        fifth_row = [delta_a_plots, delta_r_plots, delta_e_plots, delta_t_plots]
        plots = [first_row,
                 second_row,
                 third_row,
                 fourth_row,
                 fifth_row
                 ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('true_state', ['pn', 'pe', 'h', 'Va', 'alpha', 'beta', 'phi', 'theta', 'chi',
                                                        'p', 'q', 'r', 'Vg', 'wn', 'we', 'psi', 'bx', 'by', 'bz',
                                                        'aileron', 'rudder', 'elevator', 'thrust'])
        self.plotter.define_input_vector('estimated_state', ['pn_e', 'pe_e', 'h_e', 'Va_e', 'alpha_e', 'beta_e',
                                                             'phi_e', 'theta_e', 'chi_e', 'p_e', 'q_e', 'r_e',
                                                             'Vg_e', 'wn_e', 'we_e', 'psi_e', 'bx_e', 'by_e', 'bz_e'])
        self.plotter.define_input_vector('commands', ['h_c', 'Va_c', 'phi_c', 'theta_c', 'chi_c','e_c','q_c','t_c'])
        # plot timer
        self.time = 0.

    def update(self, true_state, estimated_state, commanded_state, delta_input_current, ts):
        # increment time
        self.time = ts
        commands = [commanded_state.altitude_command,  # h_c
                    commanded_state.airspeed_command,  # Va_c
                    0,  # phi_c
                    0,  # theta_c
                   0,    # chi_c
                    0,
                    0,
                    0]

        # Add the state data in vectors
        # the order has to match the order in lines 72-76
        true_state_list = [true_state.pn, true_state.pe, true_state.h,
                           true_state.Va, true_state.alpha, true_state.beta,
                           true_state.phi, true_state.theta, true_state.chi,
                           true_state.p, true_state.q, true_state.r,
                           true_state.Vg, true_state.wn, true_state.we, true_state.psi,
                           true_state.bx, true_state.by, true_state.bz, delta_input_current[0][0],
                           delta_input_current[2][0], delta_input_current[1][0],
                           delta_input_current[3][0]]
        estimated_state_list = [estimated_state.pn, estimated_state.pe, estimated_state.h,
                                estimated_state.Va, estimated_state.alpha, estimated_state.beta,
                                estimated_state.phi, estimated_state.theta, estimated_state.chi,
                                estimated_state.p, estimated_state.q, estimated_state.r,
                                estimated_state.Vg, estimated_state.wn, estimated_state.we, estimated_state.psi,
                                estimated_state.bx, estimated_state.by, estimated_state.bz]
        self.plotter.add_vector_measurement('true_state', true_state_list, self.time)
        self.plotter.add_vector_measurement('estimated_state', estimated_state_list, self.time)
        self.plotter.add_vector_measurement('commands', commands, self.time)

        # Update and display the plot
        self.plotter.update_plots()
