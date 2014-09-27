# -*- mode: ruby -*-
# vi: set ft=ruby :

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|

  config.vm.box = "precise64"
  config.vm.box_url = "http://files.vagrantup.com/precise64.box"
  config.vm.box_download_checksum_type = "sha256"
  config.vm.box_download_checksum = "9a8bdea70e1d35c1d7733f587c34af07491872f2832f0bc5f875b536520ec17e"

  config.vm.provider :vmware_fusion do |vmware, override|
    override.vm.box = "precise64_fusion"
    override.vm.box_url = "http://files.vagrantup.com/precise64_vmware_fusion.box"
    config.vm.box_download_checksum_type = "sha256"
    config.vm.box_download_checksum = "b79e900774b6a27500243d28bd9b1770e428faa3d8a3e45997f2a939b2b63570"
    vmware.vmx["memsize"] = "1024"
    vmware.vmx["numvcpus"] = "2"
  end

  config.vm.provider :virtualbox do |vb|
    vb.customize ["modifyvm", :id, "--usb", "on", "--usbehci", "on"]
  end

  config.vm.provision :shell, :inline => "apt-get update -q && cd /vagrant && ./setup.sh"
  config.vm.network "private_network", ip: "192.168.10.200"
  config.vm.network :forwarded_port, guest: 22, host: 1234
  config.ssh.forward_agent = true

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
