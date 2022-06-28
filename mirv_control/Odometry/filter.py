    
from signal import sigpending


class KalmanFilter:
    def __init__(self, mu, sig):
        self.mu = mu
        self.sig = sig

    def predict_position(self, mu_in, sig_in):
        self.mu = (self.mu * sig_in + mu_in * self.sig) / (self.mu + mu_in)
        self.sig = 1 / (1 / self.sig + 1 / sig_in)

    def predict_motion(self, mu_in, sig_in):
        self.mu += mu_in
        self.sig += sig_in

    def update(self, pos_mu, pos_sig, mot_mu, mot_sig):
        self.predict_position(pos_mu, pos_sig)
        self.predict_motion(mot_mu, mot_sig)