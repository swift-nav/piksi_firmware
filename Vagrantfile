# -*- mode: ruby -*-
# vi: set ft=ruby :

VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  config.vm.box = "trusty64"
  config.vm.box_url = "http://cloud-images.ubuntu.com/vagrant/trusty/current/trusty-server-cloudimg-i386-vagrant-disk1.box"
  config.vm.box_download_checksum_type = "sha256"
  config.vm.box_download_checksum = "1362fff999ba5dd8332e88a906de8eec7baaca6b288223dbc9a0f762a67f685b"
  config.vm.network "private_network", ip: "192.168.10.200"
  config.vm.network :forwarded_port, guest: 22, host: 1234
  config.ssh.forward_agent = true
  config.vm.provider :virtualbox do |vb|
    vb.customize ["modifyvm", :id, "--usb", "on", "--usbehci", "on"]
    vb.memory = 1024
    vb.cpus = 2
  end
  config.vm.provision "ansible" do |ansible|
    ansible.playbook = "setup/ansible/provision.yml"
    ansible.verbose = "v"
    ansible.host_key_checking = false
  end
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
