# -*- mode: ruby -*-
# vi: set ft=ruby :

VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  # Box definitions for different targets
  config.vm.define "trusty" do |trusty64|
    trusty64.vm.box = "trusty64"
    trusty64.vm.box_url = "http://cloud-images.ubuntu.com/vagrant/trusty/current/trusty-server-cloudimg-i386-vagrant-disk1.box"
    trusty64.vm.box_download_checksum_type = "sha256"
    trusty64.vm.box_download_checksum = "1362fff999ba5dd8332e88a906de8eec7baaca6b288223dbc9a0f762a67f685b"
  end
  config.vm.define "utopic" do |utopic|
    utopic.vm.box = "utopic"
    utopic.vm.box_url = "http://cloud-images.ubuntu.com/vagrant/utopic/current/utopic-server-cloudimg-i386-vagrant-disk1.box"
    utopic.vm.box_download_checksum_type = "sha256"
    utopic.vm.box_download_checksum = "0c37d6eaeeade5f4057a08de6c858513d61adc462058285bccc22ea1bac87ff9"
  end
  config.vm.define "wheezy" do |wheezy|
    wheezy.vm.box = "wheezy"
    wheezy.vm.box_url = "https://github.com/jose-lpa/packer-debian_7.6.0/releases/download/1.0/packer_virtualbox-iso_virtualbox.box"
  end
  # Networking and memory configuration
  config.vm.network "private_network", ip: "192.168.10.200"
  config.vm.network :forwarded_port, guest: 22, host: 1234
  config.ssh.forward_agent = true
  config.vm.provider :virtualbox do |vb|
    vb.memory = 1024
    vb.cpus = 2
  end
  # Provisioner definitions
  config.vm.provision "ansible" do |ansible|
    ansible.playbook = "setup/ansible/provision.yml"
    ansible.verbose = "v"
    ansible.host_key_checking = false
  end
  # Shared directory
  share_prefix = "share-"
  Dir['../*/'].each do |fname|
    basename = File.basename(fname)
    if basename.start_with?(share_prefix)
      mount_path = "/" + basename[share_prefix.length..-1]
      puts "Mounting share for #{fname} at #{mount_path}"
      config.vm.synced_folder fname, mount_path
    end
  end
end
