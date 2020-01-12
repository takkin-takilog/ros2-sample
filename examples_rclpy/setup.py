from setuptools import setup

package_name = 'examples_rclpy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='takkin',
    maintainer_email='takkin.takilog@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={  # 実行コマンド名とその呼び出し先
        'console_scripts': [
            'publisher = ' + package_name + '.publisher:main',
            'subscriber = ' + package_name + '.subscriber:main',
            'single_th = ' + package_name + '.thread_main:main_single',
            'multi_th = ' + package_name + '.thread_main:main_multi',
        ],
    },
)
