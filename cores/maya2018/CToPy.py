fmt = """
#define FunCToPy{arg_count}(fun, {types})\\
PyObject* fun(PyObject* self, PyObject* args){kl}\\
    {type_values}\\
    fun({args});\\
    return c_to_py({result});\\
{kr}\\
"""
kl = "{"
kr = "}"

max_arg_count = 10

for arg_count in range(1, max_arg_count, 1):
    types = ", ".join("t{0}".format(i) for i in range(arg_count))
    type_values = "".join("TV(t{0}, {0})".format(i) for i in range(arg_count))
    args = ",".join("v{0}".format(i) for i in range(arg_count))
    result = "v{0}".format(arg_count - 1)
    print fmt.format(**locals())


