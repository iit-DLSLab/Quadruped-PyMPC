import os
import subprocess
from setuptools import setup
from setuptools.command.install import install


class CustomInstallCommand(install):
    def run(self):
        # Ensure git submodule is updated
        submodule_path = 'quadruped_pympc/acados'  # TODO: can we get this dynamically from git?

        subprocess.check_call(['git', 'submodule', 'update', '--init', '--recursive'])
        subprocess.check_call(['git', '-C', submodule_path, 'pull'])

        # Compile acados
        acados_dir = os.path.join(os.getcwd(), submodule_path)
        build_dir = os.path.join(acados_dir, 'build')

        if not os.path.exists(build_dir):
            os.makedirs(build_dir)

        subprocess.check_call(['cmake', '..'], cwd=build_dir)
        subprocess.check_call(['make', 'install', '-j4'], cwd=build_dir)

        # Install acados Python package
        subprocess.check_call(['pip', 'install', '-e', './interfaces/acados_template'], cwd=acados_dir)

        # Set environment variables
        # TODO: This is quite invasive, we should tell users we will do this and ideally avoid writing to bashrc
        bashrc_path = os.path.expanduser('~/.bashrc')
        with open(bashrc_path, 'a') as bashrc:
            bashrc.write(f'\nexport LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"{acados_dir}/lib"')
            bashrc.write(f'\nexport ACADOS_SOURCE_DIR="{acados_dir}"')

        if os.name == 'posix':
            os.environ['DYLD_LIBRARY_PATH'] = f'{os.environ.get("LD_LIBRARY_PATH", "")}:{acados_dir}/lib'

        os.environ['ACADOS_SOURCE_DIR'] = acados_dir

        super().run()


setup(
    cmdclass={
        'install': CustomInstallCommand,
        },
    )
