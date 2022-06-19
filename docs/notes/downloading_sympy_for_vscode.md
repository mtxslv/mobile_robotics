these notes aims to register how I made sympy to work with VSCode.

1. Following the [installation guide](https://docs.sympy.org/latest/guides/getting_started/install.html#git) I git-cloned updated, and installed the library somewhere in my local machine
2. I found out where my python libraries were installed
3. I moved the `sympy/` folder into the python libraries' folder.
4. I updated sympy again (once it was in the python libraries' folder)
5. I changed  the Jupyter kernel to pyenv python 3.10.2 and reloaded the folder. Then it worked

Honestly, I don't know why once the kernel was changed the library was found. AFAIK, the library was not installed in the given kernel version and its path was known to the interpreter from the beginning (even when it was not working).

Whatever... it works and that is what matters.