# Training Specifications

[records.json](records.json) maps the six frozen policy IDs to initialization, optimizer, data, sampling, labeling and source records.

The specifications use a frozen Isaac-reference initialization, fresh optimizer, batch size 4 and 12,000 updates. The cosine schedule has 100 warm-up updates and learning rates from 0.00001 to 0.0000025. Action loss weights are 1.0 for the first seven channels and 0.1 for the remaining 25.

| Backend | Seed | TRAIN episodes | TRAIN frames |
| --- | ---: | ---: | ---: |
| Isaac Sim | 1 | 600 | 236,938 |
| MuJoCo | 0 | 600 | 186,596 |
| PyBullet | 0 | 600 | 186,714 |
| Drake | 0 | 600 | 186,314 |
| Newton | 0 | 600 | 192,884 |
| Gazebo | 0 | 600 | 186,800 |

The records identify platform-specific data views, phase sampling, hold labels and source fingerprints. Frozen inference uses the normalization file paired with each released checkpoint.
