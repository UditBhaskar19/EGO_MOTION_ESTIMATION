from matplotlib import pyplot as plt
from main_single_radar_kf import ego_motion_single_radar_kf
from main_single_radar_ols import ego_motion_single_radar_ols
from plot_functions import compare_kf_and_ols

if __name__ == '__main__':

    scene = '105'  # 105, 108
    
    print('Generating ols outputs: ')
    outputs1_ols, _ = ego_motion_single_radar_ols(scene, 'radar1', verbose=False)
    print('radar1 ols run complete !!..')
    outputs2_ols, _ = ego_motion_single_radar_ols(scene, 'radar2', verbose=False)
    print('radar2 ols run complete !!..')
    outputs3_ols, _ = ego_motion_single_radar_ols(scene, 'radar3', verbose=False)
    print('radar3 ols run complete !!..')
    outputs4_ols, _ = ego_motion_single_radar_ols(scene, 'radar4', verbose=False)
    print('radar4 ols run complete !!..') 

    print('=====================================')
    print('Generating kf outputs: ')
    outputs1_kf, _ = ego_motion_single_radar_kf(scene, 'radar1', verbose=False)
    print('radar1 kf run complete !!..')
    outputs2_kf, _ = ego_motion_single_radar_kf(scene, 'radar2', verbose=False)
    print('radar2 kf run complete !!..')
    outputs3_kf, _ = ego_motion_single_radar_kf(scene, 'radar3', verbose=False)
    print('radar3 kf run complete !!..')
    outputs4_kf, _ = ego_motion_single_radar_kf(scene, 'radar4', verbose=False)
    print('radar4 kf run complete !!..')

    compare_kf_and_ols(outputs1_kf, outputs2_kf, outputs3_kf, outputs4_kf, \
                       outputs1_ols, outputs2_ols, outputs3_ols, outputs4_ols, \
                       scene)
    plt.show()