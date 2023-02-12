from PIL import Image

foo = Image.open('4050.png')
resizeconst = 0.25
foo = foo.resize((int(foo.size[0] * resizeconst), int(foo.size[1] * resizeconst)), Image.ANTIALIAS)
foo.save('1013.png', quality=resizeconst * 100)