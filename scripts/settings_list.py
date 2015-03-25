import yaml
import pprint


class SettingsList():
  list_of_dicts= list();

  def sl_print(self):
    pp = pprint.PrettyPrinter(indent=2)
    pp.pprint(self.list_of_dicts)

  def sl_get_dict(self, group, name):
    for element in self.list_of_dicts:
        if element['name'] == name and element['group'] == group:
          return element

  def sl_get_field(self, group, name,field):
    thisdict = self.sl_get_dict(group, name)
    returnvar = ""
    if thisdict and isinstance(thisdict, dict):
      returnvar = thisdict.get(field,"")
    else:
      print "Error in settings list parsed yaml file."
      print "No dictionary dictionary for name  {0} and group is {1}".format(name,group)
    if not returnvar:
      returnvar = ""
    return returnvar

  def __init__(self,path_to_file):
    stram = open(path_to_file, "r")
    self.list_of_dicts = yaml.load(stram)
    print "Loaded settings yamle file from path " + path_to_file
    print "Number of settings loaded {0}".format(len(self.list_of_dicts))

