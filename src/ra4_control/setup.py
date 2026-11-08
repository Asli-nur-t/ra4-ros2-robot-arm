from setuptools import find_packages, setup

package_name = 'ra4_control'

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
    description='RA4 duz kinematik, hata analizi, hareket dizisi ve sensor gorsellestirme node\'lari',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'ee_pose_publisher = ra4_control.ee_pose_publisher:main',
            'fk_eval = ra4_control.fk_eval:main',
            'sequence = ra4_control.sequence:main',
            'visualizer = ra4_control.visualizer:main',
            'fk_calc = ra4_control.kinematics:main',
        ],
    },
)
