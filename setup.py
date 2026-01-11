from setuptools import find_packages, setup

package_name = 'arm_ik'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/ComputeIK.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dakshesh',
    maintainer_email='daksheshnankani1405@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik_service = arm_ik.ik_service:main',
            'ik_move_client = arm_ik.ik_move_client:main',
            'cartesian_demo = arm_ik.cartesian_demo:main'
        ],
    },
)
