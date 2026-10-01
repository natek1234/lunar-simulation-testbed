# Vendored assets — provenance

All files below are byte-exact copies of their upstream at the pinned commit.
Regenerate the digest block with `scripts/vendor_assets.py ledger`; never hand-edit a
vendored file (a drift is a red build via `tests/test_assets.py`).

| package | upstream | commit / version | license |
|---|---|---|---|
| `assets/mpo700` | `FunanShi/swerve-nav` `robot/mpo_700.xml`, `robot/meshes/` (Neobotix MPO-700 from `neo_simulation2`) | `0fb97e36dda5d072f4e4cc6cce87ca1f6a98cfe0` | MIT (`assets/mpo700/LICENSE`) |
| `assets/franka_fr3` | `google-deepmind/mujoco_menagerie` `franka_fr3/` (arm only; `scene.xml` and PNGs excluded) | `c1a4eeb85694ae1dffe33ff1797d4e528928a133` | Apache-2.0 (`assets/franka_fr3/LICENSE`) |
| `assets/franka_hand` | same commit, `franka_emika_panda/hand.xml` + its meshes (the FR3 package ships no hand) | `c1a4eeb85694ae1dffe33ff1797d4e528928a133` | Apache-2.0 (`assets/franka_hand/LICENSE`) |
| `assets/textures` | Poly Haven `moon_01`, `moon_02` diffuse 1k | 2026-09 download | CC0 1.0 |

Notes that matter when composing (`docs/design.md`'s measured MuJoCo pitfalls section):
`mpo_700.xml` sets `<option timestep="0.005">`
and `<compiler meshdir="meshes/">`; `hand.xml`'s root body carries `quat="0 0 0 1"` and
upstream mounts it at Rz(−45°); FR3 and the hand both define materials named `white` and
`black`, so the hand must be attached with a prefix.

## sha256

```
e30a69784e5fd7bb6e879769b31b9c47fb701cf2342843aec03485601613748f  assets/franka_fr3/assets/link0.obj
dfc6d94330de8ddb005b311bfdba9f3b8e1aa7c256b71592ee7ff32cb9a9a5aa  assets/franka_fr3/assets/link0.stl
c830c25874a8a5736580cfce6f247fd145782d0b050fa424b00dd8194d8b2872  assets/franka_fr3/assets/link0_0.obj
12f0affb9b540574f28ddf579413455a4728a093498457c9408539c31e2efcb8  assets/franka_fr3/assets/link0_1.obj
19a4d2fd6fef4fa09e109349bdaa315a00a00b17c814392902665bd92217f94c  assets/franka_fr3/assets/link0_2.obj
ed22c0de7adbc10618321715e047d1d9a36acca3260bc9497acd3ebda998e4e0  assets/franka_fr3/assets/link0_3.obj
bc99c6e6b4752daa010ec2ac4ab38457eca0a1b75fcb4a60139b8200e186454d  assets/franka_fr3/assets/link0_4.obj
dbccb46d8a3bfd602cd110a0bf69036e42661ec89c0074f18dc10dbdd2c0325e  assets/franka_fr3/assets/link0_5.obj
9205596427463353249ab5acc7c7b4f5eb8be358527cd9e229b656b16f5bd1fe  assets/franka_fr3/assets/link0_6.obj
a9e73987b774f3fb76ec640c2f6e172b128736363792f1697a13d7795f745663  assets/franka_fr3/assets/link1.obj
e41a39a94108fcf56aacff603fc91ec80541f4c1af17b51a0de5617f5566e6d2  assets/franka_fr3/assets/link1.stl
1818f1eed57552d006a3e9353a670a39c271ac67833274efcee2403db7e4fa92  assets/franka_fr3/assets/link2.obj
370f7605a0fae3529db169ded50f52f171024aa792d4d773bc84197301f6a039  assets/franka_fr3/assets/link2.stl
20a2e2adfdeaf9ff775f0fba0ceadbbc7cc7ead2695759157e5f1ef1d1793e57  assets/franka_fr3/assets/link3.obj
0a8d638b9349c6c0eefc4e888636ac4838c4b27170f18a51699321118af709c1  assets/franka_fr3/assets/link3.stl
d15c6360be859d55ed1e0d5b08317c4c26d5b3a2a4743c04faf4cc07080f5c9a  assets/franka_fr3/assets/link3_0.obj
89c2c02bdb1c5175d42c5181b4dcc0bb282fab1c5a0c2f99183c156ad45c2f6d  assets/franka_fr3/assets/link3_1.obj
ee1c2138efa48c4a4a355714138360bca4a447e1b41e04ed027979990c806ef1  assets/franka_fr3/assets/link4.obj
0180ebb5772ec9840cb049750cffb29a9ddc90311752a16ea34757782ef9e48d  assets/franka_fr3/assets/link4.stl
8da93d2b65258890be3dfaa39f3cef5acd3d85e7e4e515eb6eb7e1284c3af795  assets/franka_fr3/assets/link4_0.obj
0428a185f74edb3a23c81d71adc9269c8b63f73805f9d210960a5d425fee060d  assets/franka_fr3/assets/link4_1.obj
bb2d78050b1adf67d437762a0dcdf2e4a18b619069a0cdaefc0c44444451ba29  assets/franka_fr3/assets/link5.obj
dd17e688c7870e722283525879643d53a74c0024d328b0e14b034b54c8b6c31a  assets/franka_fr3/assets/link5.stl
bcc32346b6558d8a1c42594b6988d4f6f68ece2e0a769c28b0242d5b398e1f86  assets/franka_fr3/assets/link5_0.obj
d2ef3af3be96ebb7c3f78cf3cedbdfb36b375db8dc32068c5dca613d9086e5e5  assets/franka_fr3/assets/link5_1.obj
a897831482eed642e5d441d1a1abf0ede89d37672f8168ce2cd40d620e25daca  assets/franka_fr3/assets/link5_2.obj
93743fd1dad97da9ec56f3ef2c08b7930f54402951f9eba3fdbc9018d85d314b  assets/franka_fr3/assets/link6.obj
20b768e99a0e0440b5754dcca108016434e57937cc356acd9c352ccd3cb27f77  assets/franka_fr3/assets/link6.stl
f2a77f8890b3c24de62c6f23c2c63e59cb95b0e3986741d00ac12ec733156635  assets/franka_fr3/assets/link6_0.obj
4165ccedf27ec576c8fe3ec86702a43cbdacfb2a25328f6762721a92d22ca8fc  assets/franka_fr3/assets/link6_1.obj
1111395c7a1c4a1c803e35969fb4cdd5a183a21292a0da5f0519f6d684083445  assets/franka_fr3/assets/link6_2.obj
86fc49381273ccc0079b1f7ce5a77ce363e85ef4df5ebcf2b07c1ae6173043e6  assets/franka_fr3/assets/link6_3.obj
ed3f2d2a05b7d7704f4d04f38b99ce59792a8ef79e2f989567a0319a9f029599  assets/franka_fr3/assets/link6_4.obj
435abeff7bded0c6e9af1e3f98857413fd3964a65c138efc9d99703d9dfffbbb  assets/franka_fr3/assets/link6_5.obj
b76323550be892e32482f5dee5ca0631b080cf6543404495961834de19f89e71  assets/franka_fr3/assets/link6_6.obj
5189566a36197a91792e8704f9dd0097e63a1141c5aca72427fb977d4389c72a  assets/franka_fr3/assets/link6_7.obj
7a6304915e21db77a78e22b30c97ce923ee7cdd1ddb6e30cfb6d048b4c03293e  assets/franka_fr3/assets/link7.obj
92ac6afcf7574c034d3170d8a68e95ac9048ab9d0dd5bbd8311b86e551b9ab1c  assets/franka_fr3/assets/link7.stl
bc0953056e78a063c0de7c81f7ba320b74f7957ee5e7da0786e678be64769ab2  assets/franka_fr3/assets/link7_0.obj
bab975d59b724a0fac020d80c4483b77d29b9217485f0aae0b866a1b70b56fad  assets/franka_fr3/assets/link7_1.obj
7f7b27bab891757731d78c0b717a1da3fc48cebb6db5d3cd37994d768ce9b509  assets/franka_fr3/assets/link7_2.obj
3ccfdcf6043602577da37969244a660ebd72bc4bd172b7fbeb542809b62dccff  assets/franka_fr3/assets/link7_3.obj
a7c798743420ab2d01b35d0cb0714d40328e1babfb339afae0b527eb1f569693  assets/franka_fr3/CHANGELOG.md
8f33d865ea6fb9596bea5cc242c8de43dbf3dd372e323b44e229e7840fecd47d  assets/franka_fr3/fr3.xml
d213a10eac9efe085d088824559cf30f34733fee1b232cefbd1952b99fcfdd78  assets/franka_fr3/LICENSE
19c8ad2e78fb0d046bd9db23a64448a81f046d5b4fe0bdd0eeebeec6759fab41  assets/franka_fr3/README.md
215098c0fcdf7192f6c503a37cf49e3efecf8a9e1320f02b17c5584d89cc5b2c  assets/franka_hand/assets/finger_0.obj
cf239e9aca26bba09dbbf1c544c47d87cfc9a2f0637c8bb786af750b6d4eca44  assets/franka_hand/assets/finger_1.obj
94493e94f30fe940f2c8ca2f155c3bbe67bbff406d3edf5e261670d2f0f6e2ed  assets/franka_hand/assets/hand.stl
b94aab901086f85df04c019b30f43bf9aa23a7ad0443ec80a8c14662a41cabd2  assets/franka_hand/assets/hand_0.obj
cd76f7494bb314406a1819c46eb826ee254894f87235714f604a470653d9badb  assets/franka_hand/assets/hand_1.obj
e0f44a71712afa5b98eb20355eff17763d3e2e19f459b2bef19abed3727e64b3  assets/franka_hand/assets/hand_2.obj
a431a06d99e88a1b322b3122a01424ee7704daeaad0bc343b90b9604f014ad0c  assets/franka_hand/assets/hand_3.obj
21f96ef6a8de082e41b1ea9834f1043de359e5d60305541bcb63539581f53bff  assets/franka_hand/assets/hand_4.obj
d2a6539d0fd7c55c75c14f11c78f04505ee33f31d93e6112e3ad7120e5ee786a  assets/franka_hand/hand.xml
d213a10eac9efe085d088824559cf30f34733fee1b232cefbd1952b99fcfdd78  assets/franka_hand/LICENSE
dcf360cc9999ab214968d122f34b76a4fa043867d924f5f86d1e18055f932f42  assets/mpo700/LICENSE
0d90fe7d688a6da0b82b0f8053231a1c8986de34aa839f8113ed53a610c09227  assets/mpo700/meshes/material.mtl
38c0fd6d954af105161a9093959c3701ac759a01052c0c0a3135cc3675fad918  assets/mpo700/meshes/material_0.png
53651d2a3a22aea5cac7cbdb5ef7863d5f7a68ed7a9507ca593d2b3dac4dfb2b  assets/mpo700/meshes/MPO-700-BODY.obj
c22e3fad1f91c6cd4226f390c69120757dadec87755c3d1c94d8466b830fff55  assets/mpo700/meshes/MPO-700-HEAD.obj
a430c095fcb5b994396daa1f4d49cf575c99f443cf8bb18317faa45fd9b6430a  assets/mpo700/meshes/MPO-700-WHEEL.obj
9b8d3d55df80d270069f56ef23259d16a39b2074fb3a5d372b86bc08c69886be  assets/mpo700/mpo_700.xml
8b0cab6d74bb5f086816aefab2eceaf42a840e2a744e570b2d3a456fbf793e50  assets/textures/moon_01_diff_1k.png
d33c7fe793c3bf014df871cf353e64775ce61a17feaf501ab98c28299e19c05a  assets/textures/moon_02_diff_1k.png
```
