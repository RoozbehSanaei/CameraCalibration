## Postponed type hints (`from __future__ import annotations`)
- The file includes `from __future__ import annotations` near the top (after the module docstring), which tells Python to store type hints in a “later” form when it creates functions and classes, instead of trying to fully resolve every name in the type hints immediately.  
- In this code, many functions and classes include type hints (for example, hints for inputs and outputs), and some hinted names are introduced later in the file; with this setting, Python can still build those functions and classes first and keep the hints as attached notes.  
- Overall benefit: the program can keep clear type notes without Python tripping over type-hint names that appear later in the file.

## Type-alias marker (`TypeAlias`)
- The code brings in the marker with `from typing import TypeAlias`, then uses it in lines like `Vec: TypeAlias = np.ndarray` so Python binds `Vec` to the same thing as `np.ndarray` at runtime.  
- In the rest of the file, names like `Vec` and `Mat` are used in type hints; Python treats them as normal names, while tools that read type hints can also see that these names were intended to represent “type names.”  
- Overall benefit: the code can use short, consistent names in type hints while still pointing to the same underlying runtime objects.

## “Do not reassign” marker (`Final`)
- The code imports the marker using `from typing import Final`, and then places it in type-hint positions to mark certain names as intended to stay unchanged.  
- Python still treats the value as an ordinary variable at runtime, meaning nothing stops reassignment unless the code itself forbids it; the “final” meaning is stored as metadata that other tools can read.  
- Overall benefit: readers and type-checking tools can understand which names are meant to behave like constants.

## Dataclass decorator (`@dataclass(...)`)
- The file imports the decorator with `from dataclasses import dataclass`, then places `@dataclass(...)` above classes so Python first creates the class and then runs the decorator on that class object.  
- In this code, the decorator reads the field definitions from the class’s type hints and auto-creates common methods (especially the constructor), which is why the program can create instances by calling the class with field values.  
- Overall benefit: the classes that hold calibration parameters can be defined with far less boilerplate.

## Dataclass immutability switch (`frozen=True`)
- The code passes `frozen=True` inside `@dataclass(frozen=True, ...)`, which changes how Python handles attribute setting on instances after they are created.  
- In this file, once an instance holding parameters is built, later code can read fields, yet normal assignments to those fields are blocked by the object’s attribute-setting rules that dataclasses install.  
- Overall benefit: the parameter objects behave more like “fixed records,” which reduces accidental edits while the program runs.

## Dataclass slot storage (`slots=True`)
- The code passes `slots=True` inside `@dataclass(..., slots=True)`, which changes how Python stores fields inside each instance.  
- In this file, instances store a fixed set of named fields using a slot-based layout rather than a per-instance attribute dictionary, so attribute access is handled through those fixed slots.  
- Overall benefit: instances become more constrained in what attributes they can hold, which reduces accidental “new attribute” mistakes.

## Abstract “ordered collection” type (`Sequence`)
- The code imports it via `from collections.abc import Sequence`, then uses it in function annotations to describe inputs that act like an ordered list of items.  
- In this file, a function that accepts argument tokens is annotated with `Sequence[str] | None` to show it expects something that can be iterated and treated like an ordered group of strings.  
- Overall benefit: the code’s expectations about “what kind of container comes in” are clearer to readers.

## “Either/or” type hints (`|` in annotations)
- The code uses the `|` symbol in type hints (for example, `Sequence[str] | None`) to record “this can be one thing or another” in the annotation metadata.  
- In this file, parameters that may be omitted are annotated this way, and the runtime behavior still depends on regular checks for `None`; the `|` form just records that intent in the function’s attached type notes.  
- Overall benefit: the type hints match the code path that handles “provided value” versus “missing value.”

## Computed attribute wrapper (`@property`)
- The code uses `@property` on a method in the intrinsics class so that later access looks like a field read, even though it runs a function.  
- In this file, reading something like `K.K` triggers Python’s attribute lookup to call the property’s getter, which builds and returns the camera matrix from stored numeric fields.  
- Overall benefit: the rest of the code can use a clean “field-like” access pattern for values that are actually computed.

## Formatted strings (f-strings)
- The code builds readable output using f-strings, which are strings that contain embedded expressions inside `{...}` that Python evaluates at runtime.  
- In this file, when results are printed or logged, Python evaluates the expressions inside the f-string, applies formatting rules like fixed decimal places, and then produces a normal final string.  
- Overall benefit: numeric results are turned into consistent, human-readable text without manual string building.

## Assign-and-use in one expression (`:=`)
- The code uses `:=` so Python can compute a value, store it in a name, and also use it immediately in a condition.  
- In this file, a computed number (such as a size or “amount” value) is saved into a variable during an `if` test, and the same computed value is used right away for the comparison.  
- Overall benefit: the code avoids computing the same value twice while still keeping the logic in one place.

## Paired iteration (`zip(..., strict=True)`)
- The code uses `zip(..., strict=True)` to iterate over two same-length sequences side by side, where Python pulls one item from each sequence per step.  
- In this file, the strict setting means Python checks that both sequences end together; if one sequence is longer, iteration raises an error instead of silently stopping early.  
- Overall benefit: mismatched paired data is caught immediately during the loop rather than producing partial, misleading processing.

## Counting while looping (`enumerate(...)`)
- The code uses `enumerate(...)`, where Python wraps an iterator and adds an internal counter that starts at zero by default.  
- In this file, when iterating over multiple “views,” the loop receives both the view index and the view data in each iteration step, without manual counter code.  
- Overall benefit: the loop gets an index in a clean way, reducing bookkeeping mistakes.

## List comprehension (`[ ... for ... ]`)
- The code uses a list comprehension to build a list in one expression, where Python internally runs loops and appends each produced element to a new list.  
- In this file, it generates a grid of planar points by looping over two ranges and producing a pair of coordinates for each grid position, then later turns that list into a NumPy array.  
- Overall benefit: the “build a list from a loop” pattern is compact and easy to follow once you know the form.

## Generator expression with `next(...)`)
- The code uses a generator expression inside `next(...)`, where Python creates a “produce items one at a time” object rather than building a full list.  
- In this file, `next(...)` pulls just the first item that matches a condition, and if nothing matches, it returns a default value instead of failing.  
- Overall benefit: the code can “find the first match” without creating extra intermediate containers.

## Exception handling (`try` / `except`)
- The code uses `try: ... except ...:` so Python can attempt an operation and, if a named error occurs, jump to a fallback path.  
- In this file, a linear-algebra operation may fail for certain inputs; when it raises the specific error, Python transfers control to the `except` block where an alternative routine is run.  
- Overall benefit: the program can keep running and produce a result even when a particular calculation path fails.

## Matrix multiply operator (`@`)
- The code uses the `@` operator, which Python treats as a special “matrix multiplication” operation by calling the object’s internal matrix-multiply method.  
- In this file, arrays are combined with `@` to apply transforms and produce new point arrays, and the heavy numerical work is performed by the array library’s implementation behind that operator.  
- Overall benefit: the math reads like “matrix times matrix,” which keeps the transformation steps clear.

## Range slicing that keeps a dimension (`2:3` slicing)
- The code uses slicing like `2:3` in array indexing, where Python creates a “slice object” and passes it into the array’s indexing logic.  
- In this file, choosing `2:3` (a one-step range) keeps that axis as a one-wide dimension, which affects how later division spreads across columns when the program normalizes coordinates.  
- Overall benefit: the code controls array shapes in a predictable way during calculations.

## Import aliasing (`import numpy as np`)
- The file uses `import numpy as np`, where Python loads the module and binds it to the short name `np` in this file.  
- In this code, later references like `np.array(...)` and `np.linalg...` are regular attribute lookups on that bound module name, which is how the program reaches array creation and math routines.  
- Overall benefit: the code stays readable and consistent by using a short, standard module name.

## Script entry check (`if __name__ == "__main__":`)
- The file ends with `if __name__ == "__main__":`, where Python uses the special name `__name__` to know whether the file is being run directly or imported.  
- In this code, when run directly, Python executes the block and calls `main()` to run the test pipeline; when imported, Python skips that block while still providing the functions and classes.  
- Overall benefit: the same file can act as both a reusable module and a runnable program.

## Logging setup and use (`import logging`, `logging.basicConfig(...)`, `logging.getLogger(...)`)
- The code imports logging with `import logging`, configures it once using `logging.basicConfig(...)`, then obtains a logger via `logging.getLogger(...)` and uses it for messages.  
- In this file, each log call causes Python to create a “message record” and pass it through the configured output rules so it prints in a consistent format and level.  
- Overall benefit: status messages are handled in a structured way rather than scattered, inconsistent prints.
