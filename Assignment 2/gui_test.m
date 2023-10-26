close all;
clear all;
clc;

sys = FruitLoopSystem();
sys.beginFruitLoop();

[tree1_pos, tree1_obj, tree1_verts, tree1_picked, tree2_pos, tree2_obj, tree2_verts, tree2_picked, tree1_crate_pos, tree2_crate_pos, tree1_above_crate, tree2_above_crate, tree1_sorted_crate_pos
, tree2_sorted_crate_pos]