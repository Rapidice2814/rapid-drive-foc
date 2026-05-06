import h5py


with h5py.File('debug_log.h5', 'r') as f:
    print(f['time'][:5])
    print(f['ab_current.alpha'][:5])