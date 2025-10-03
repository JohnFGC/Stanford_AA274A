#!/usr/bin/env python3

# add import and helper functions here
import numpy as np

if __name__ == "__main__":
    print("Hello World")
    
    np.random.seed(42)
    A = np.random.normal(size=(4, 4))
    B = np.random.normal(size=(4, 2))
    print(A @ B)
    
    np.random.seed(42)
    x = np.random.normal(size=(4, 10))
    y = np.square(x[None, :, :] - x[:, None, :])
    z = np.sum(y, axis=2)
    print(z)
    