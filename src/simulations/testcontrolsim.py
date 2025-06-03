from simulations.controlsim import ControlSim
class test_ControlSim(ControlSim):
    def __init__(self,network,taskparams,actuators,controlparams = {}):
        super().__init__(network=network,taskparams=taskparams,actuators=actuators,controlparams=controlparams)

    def compute_input(self, k, sim_period, demand, controller, controller_name, uData, yData, ypredData, m, p, n_regions, r, u_min, u_max):
        return controller.get_next_input()
