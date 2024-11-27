#!/usr/bin/env python3

default_values = {
    "Model": [
        "ARAP_depth_onlyTriang", "ARAP_depth", "ARAP_NoGlobal", 
        "ARAP", "Elastic", "HyperElasticOdgen", "ARAP_OneSet"
    ],
    "Triangulation": ["InRays", "TwoPoints", "FarPoints"],
    "Depth": [20, 80, 150],
    "Shape": ["Planar", "Gradual"],
    "ExperimentType": range(1, 7),
    "Experiment": range(1, 6),
}

shape_experiment_types = {
    "Planar": [1, 2, 3, 4, 5, 6],
    "Gradual": [2, 3, 5, 6],
}