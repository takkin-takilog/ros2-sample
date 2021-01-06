from setuptools import setup

package_name = 'rclpy_sample'

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
    entry_points={
        'console_scripts': [
            'publisher = ' + package_name + '.topic.publisher:main',
            'subscriber = ' + package_name + '.topic.subscriber:main',
            'single_th = ' + package_name + '.topic.thread_main:main_single',
            'multi_th = ' + package_name + '.topic.thread_main:main_multi',
            'multi_thread_snd = ' + package_name + '.callback_group.multi_thread_snd:main',
            'multi_thread_rcv = ' + package_name + '.callback_group.multi_thread_rcv:main',
            'async_service_server = ' + package_name + '.service.async_server:main',
            'async_service_client = ' + package_name + '.service.async_client:main',
            'async_action_server = ' + package_name + '.action.async_server:main',
            'async_action_client = ' + package_name + '.action.async_client:main',

            'spin = ' + package_name + '.spin.spin:main_spin',
            'spin_once = ' + package_name + '.spin.spin:main_spin_once',

        ],
    },
)
