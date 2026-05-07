import numpy as np
import h5py
import config
from datetime import datetime
from pathlib import Path

def get_log_filename():
    timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    log_dir = Path("logs")
    log_dir.mkdir(parents=True, exist_ok=True)
    print(f"mask: 0x{config.log_mask:08X}")
    return log_dir / f"debug_log_{timestamp}_{config.log_mask:08X}.h5"

def _dtype_from_values(values):
    first = values[0]
    if isinstance(first, float):
        return np.float32
    return np.uint32

def init_hdf5_file(filename, decoded, log_mask):
    with h5py.File(filename, "a") as f:
        if "time" not in f:
            f.create_dataset(
                "time",
                shape=(0,),
                maxshape=(None,),
                dtype=np.uint32,
                chunks=True
            )

        for name, values in decoded["signals"].items():
            if name not in f:
                f.create_dataset(
                    name,
                    shape=(0,),
                    maxshape=(None,),
                    dtype=_dtype_from_values(values),
                    chunks=True
                )

        f.attrs["log_mask"] = int(log_mask)
        f.attrs["signal_count"] = int(decoded["signal_count"])
        f.attrs["sample_count"] = int(decoded["sample_count"])
        print(f"Initialized HDF5 file with log_mask={log_mask:08b} and signals {[sig['name'] for sig in decoded['enabled_signals']]}")


def append_decoded_batch_to_hdf5(filename, decoded_list):
    if not decoded_list:
        return

    total_samples = sum(d["sample_count"] for d in decoded_list)

    time_parts = []
    signal_parts = {}

    for decoded in decoded_list:
        sample_count = decoded["sample_count"]
        base_timestamp = decoded["timestamp"]

        time_parts.append(
            np.arange(base_timestamp, base_timestamp + sample_count, dtype=np.uint32)
        )

        for name, values in decoded["signals"].items():
            if name not in signal_parts:
                signal_parts[name] = []
            signal_parts[name].append(np.asarray(values))

    time_array = np.concatenate(time_parts)

    with h5py.File(filename, "a") as f:
        time_ds = f["time"]
        old_len = time_ds.shape[0]
        new_len = old_len + total_samples

        time_ds.resize((new_len,))
        time_ds[old_len:new_len] = time_array

        for name, parts in signal_parts.items():
            ds = f[name]
            values_np = np.concatenate(parts).astype(ds.dtype, copy=False)
            ds.resize((new_len,))
            ds[old_len:new_len] = values_np