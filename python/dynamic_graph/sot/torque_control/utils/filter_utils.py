from dynamic_graph.sot.core.filter_differentiator import FilterDifferentiator
from numpy import arctan2, imag, pi, real, unwrap
from pylab import (
    grid,
    plot,
    show,
    subplot,
    subplots_adjust,
    title,
    xlabel,
    ylabel,
    ylim,
)


def create_butter_lp_filter_Wn_05_N_2(name, dt, size):
    lp_filter = FilterDifferentiator(name)
    # from scipy.signal import butter
    # (b,a) = butter(N=2, Wn=0.05)
    # delay about 9*dt
    lp_filter.init(dt, size, (0.00554272, 0.01108543, 0.00554272), (1.0, -1.77863178, 0.80080265))
    return lp_filter


def create_butter_lp_filter_Wn_05_N_3(name, dt, size):
    lp_filter = FilterDifferentiator(name)
    # (b,a) = butter(N=3, Wn=0.05)
    # delay about 13*dt
    lp_filter.init(
        dt,
        size,
        (0.00041655, 0.00124964, 0.00124964, 0.00041655),
        (1.0, -2.6861574, 2.41965511, -0.73016535),
    )
    return lp_filter


def create_butter_lp_filter_Wn_05_N_4(name, dt, size):
    lp_filter = FilterDifferentiator(name)
    # (b,a) = butter(N=4, Wn=0.05)
    # delay about 16*dt
    lp_filter.init(
        dt,
        size,
        (
            3.12389769e-05,
            1.24955908e-04,
            1.87433862e-04,
            1.24955908e-04,
            3.12389769e-05,
        ),
        (1.0, -3.58973389, 4.85127588, -2.92405266, 0.66301048),
    )
    return lp_filter


def create_butter_lp_filter_Wn_03_N_3(name, dt, size):
    lp_filter = FilterDifferentiator(name)
    # (b,a) = butter(N=3, Wn=0.03)
    # delay about 20*dt
    lp_filter.init(
        dt,
        size,
        (9.54425084e-05, 2.86327525e-04, 2.86327525e-04, 9.54425084e-05),
        (1.0, -2.81157368, 2.64048349, -0.82814628),
    )
    return lp_filter


def create_butter_lp_filter_Wn_03_N_4(name, dt, size):
    lp_filter = FilterDifferentiator(name)
    # (b,a) = butter(N=4, Wn=0.03)
    # delay about 28*dt
    lp_filter.init(
        dt,
        size,
        (
            4.37268880e-06,
            1.74907552e-05,
            2.62361328e-05,
            1.74907552e-05,
            4.37268880e-06,
        ),
        (1.0, -3.75376276, 5.29115258, -3.3189386, 0.78161874),
    )
    return lp_filter


def create_chebi2_lp_filter_Wn_03_N_4(name, dt, size):
    lp_filter = FilterDifferentiator(name)
    # (b,a) = cheby2(4, 20, 0.03)
    # delay about 23*dt
    lp_filter.init(
        dt,
        size,
        (0.09147425, -0.35947268, 0.53605377, -0.35947268, 0.09147425),
        (1.0, -3.7862251, 5.38178322, -3.40348456, 0.80798333),
    )
    return lp_filter


def filter_series(b1, a1, b2, a2):
    import numpy as np

    b = np.polymul(b1, b2)
    a = np.polymul(a1, a2)
    return b, a


def create_chebi1_checby2_series_filter(name, dt, size):
    # b1,a1=cheby2(2, 20,0.05);
    # b2,a2 = cheby1(4,0.05,0.08);
    # (b,a) = filter_series(b1,a1,b2,a2);
    lp_filter = FilterDifferentiator(name)

    lp_filter.init(
        dt,
        size,
        (
            2.16439898e-05,
            4.43473520e-05,
            -1.74065002e-05,
            -8.02197247e-05,
            -1.74065002e-05,
            4.43473520e-05,
            2.16439898e-05,
        ),
        (
            1.0,
            -5.32595322,
            11.89749109,
            -14.26803139,
            9.68705647,
            -3.52968633,
            0.53914042,
        ),
    )
    return lp_filter


def mfreqz(fs, b, a=1, show_plot=True):
    import scipy.signal as signal

    w, h = signal.freqz(b, a)
    f = w * fs / (2 * max(w))
    # h_dB = 20 * log10(abs(h))
    ax1 = subplot(211)
    plot(f, abs(h))
    ylim(0, 1)
    ylabel("Magnitude")
    xlabel(r"Frequency (Hz)")
    title(r"Frequency response")
    grid(True)
    subplot(212, sharex=ax1)
    h_Phase = unwrap(arctan2(imag(h), real(h)))
    delay = h_Phase / (2 * pi * f)
    plot(f, delay)
    ylabel("Delay (s)")
    xlabel(r"Frequency (Hz)")
    title(r"Phase delay")
    grid(True)
    subplots_adjust(hspace=0.5)
    if show_plot:
        show()


def compare_with_savgol(fs, b, a):
    from scipy.signal import savgol_coeffs

    b_sg = savgol_coeffs(81, 2)
    mfreqz(fs, b_sg, 1, False)
    mfreqz(fs, b, a)
