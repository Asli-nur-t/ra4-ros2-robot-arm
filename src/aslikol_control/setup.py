from setuptools import find_packages, setup

package_name = 'aslikol_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aslinur Topcu',
    maintainer_email='aslinur@example.com',
    description='aslikol duz kinematik, hata analizi, hareket dizisi ve sensor gorsellestirme node\'lari',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'ee_pose_publisher = aslikol_control.ee_pose_publisher:main',
            'fk_eval = aslikol_control.fk_eval:main',
            'sequence = aslikol_control.sequence:main',
            'visualizer = aslikol_control.visualizer:main',
            'fk_calc = aslikol_control.kinematics:main',
        ],
    },
)
