import yaml
import pprint

class SettingsList():
  list_of_dicts = list();

  def __str__(self):
    pp = pprint.PrettyPrinter(indent=2)
    pp.pprint(self.list_of_dicts)

  def get_dict(self, group, name):
    for element in self.list_of_dicts:
        if element['name'] == name and element['group'] == group:
          return element

  def get_field(self, group, name,field):
    thisdict = self.get_dict(group, name)
    returnvar = ""
    if thisdict and isinstance(thisdict, dict):
      returnvar = thisdict.get(field,"")
    else:
      print "Error in settings list parsed yaml file."
      print "No entry for name  {0} and group is {1}".format(name,group)
    if not returnvar:
      returnvar = ""
    return returnvar

  def return_groups(self):
    group_set = set()
    output = []
    for element in self.list_of_dicts:
        group = element['group']
        if group not in group_set:
          output.append(element['group'])
          group_set.add(group)
    return output


  def __init__(self,path_to_file):
    stram = open(path_to_file, "r")
    self.list_of_dicts = yaml.load(stram)
    print "Loaded settings yaml file from path " + path_to_file
    print "Number of settings loaded {0}".format(len(self.list_of_dicts))

