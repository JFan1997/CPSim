import numpy as np


class ActuatorAttack:
    def __init__(self, category, param, start_index, end_index=None, simulator=None):
        """
        category: 'bias', 'delay', 'replay'  todo: 'stealthy'
        param
        """
        self.cat = category
        self.param = param
        self.start_index = start_index
        self.end_index = end_index if end_index is not None else np.inf
        self.cur_index = None
        self.disabled = False
        self.simulator = simulator

    def update_param(self, param):
        self.param = param

    def disable(self):
        self.disabled = True

    def launch(self, cur_data, cur_index, history_intact_data):
        self.cur_index = cur_index

        if self.cat == 'bias':
            if cur_index < self.start_index or cur_index > self.end_index or self.disabled:
                # param is the bias on cur_control_signal
                return cur_data +self.param
            else:
                return cur_data

        if self.cat == 'delay':
            # param is the number of delay steps
            if self.cur_index - self.start_index >= self.param:
                delayed_index = self.cur_index - self.param
                if delayed_index < 0:
                    # Handle the edge case: return start_index data or some default
                    return history_intact_data[self.start_index]
                else:
                    return history_intact_data[delayed_index]
        if self.cat == 'replay':
            # replay interval param['start'] ~ param['end']
            delta_index = self.cur_index - self.start_index
            offset_index = self.param['start'] + delta_index % (self.param['end'] - self.param['start'])
            return history_intact_data[offset_index]

    def get_C_filter(self):
        if self.cat == 'bias':
            cnt = len(self.param)
            bias_e = np.full((cnt,), False)
            for i in range(cnt):
                if self.param[i] != 0:
                    bias_e[i] = True
            C_filter = []
            for i in range(4):
                if bias_e[i]:
                    continue
                else:
                    tmp = [0, 0, 0, 0]
                    tmp[i] = 1
                    C_filter.append(tmp)
            return C_filter
        else:
            raise NotImplementedError()


if __name__ == "__main__":
    pass
    # attack1 = Attack('bias', np.array([1, 0, 0, 0]), 0)
    # attack2 = Attack('delay', np.array([1, 0, 0, 0]), 0)
    # print(attack1.get_C_filter())
