# FairFoody: Bringing in Fairness in Food Delivery

This repository contains official implementation of the algorithms defined in our paper titled, Anjali, Rahul Yadav, Ashish Nair, Abhijnan Chakraborty, Sayan Ranu and Amitabha Bagchi, "FairFoody: Bringing in Fairness in Food Delivery", in AAAI, 2022.

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
The code for simulation and algorithms defined in our paper is provided in [./code](https://github.com/idea-iitd/fairfoody/tree/main/code). An anonymized version of the proprietary dataset will be made available once an agreement is signed. Instructions to request data are available at this [link](https://www.cse.iitd.ac.in/~sayan/files/foodmatch.txt).


## Bibtex

```
@inproceedings{fairfoody,
  author = {Anjali Gupta and Shreyansh Nagori and Abhijnan Chakraborty and Rohit Vaish and Sayan Ranu and Prajit nadkarni and Narendra Varma and Muthusamy Chelliah},
  title = {Towards Fair Allocation in Social Commerce Platforms},
  booktitle = {The Web Conference (formerly known as International World Wide Web Conference, abbreviated as WWW) },
  year = {2023}
}
```
