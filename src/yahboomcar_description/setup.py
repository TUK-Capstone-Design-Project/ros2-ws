from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.*'))),

        # --- [추가] Gazebo 모델 파일들을 install 폴더로 복사 ---
        (os.path.join('share', package_name, 'models/lcode_ground'), glob('models/lcode_ground/*.sdf')),
        (os.path.join('share', package_name, 'models/lcode_ground'), glob('models/lcode_ground/*.config')),
        (os.path.join('share', package_name, 'models/lcode_ground/materials/scripts'), glob('models/lcode_ground/materials/scripts/*')),
        (os.path.join('share', package_name, 'models/lcode_ground/materials/textures'), glob('models/lcode_ground/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='Yahboom Raspbot V2 Description with LCode Ground',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
