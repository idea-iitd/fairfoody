# FairFoody: Bringing in Fairness in Food Delivery

This repository contains official implementation of the algorithms defined in our paper.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine.

### Prerequisites

The algorithms are implemented in C++11 (GCC 7.4.0) and evaluation scripts are implemented in Python 3 (Python 3.6)

### Installation

Setup a conda environment which includes packages required to run evaluation scripts:

```bash
conda env create -f environment.yml
conda activate ff_evn
```

### Datasets and evaluation procedure
We use two datasets for evaluation of algorithms:
-The code for simulation and algorithms defined in our paper is provided in [./code]. An anonymized version of the proprietary dataset is avilable at "https://drive.google.com/drive/folders/1dMpbYklf4zU4IGlR2OG5AA3nmf45_nJy?usp=sharing".

## References

## References

[1] M.  Joshi,  A.  Singh,  S.  Ranu,  A.  Bagchi,  P.  Karia,  and  P.  Kala,“Batching and matching for food delivery in dynamic road networks,”inProc. ICDE, 2021.<br>
[2] T. Suhr, A. J. Biega, M. Zehlike, K. P. Gummadi, and A. Chakraborty,“Two-sided  fairness  for  repeated  matchings  in  two-sided  markets:  Acase study of a ride-hailing platform,” inACM KDD, 2019.<br>
[3] https://github.com/idea-iitd/FoodMatch