#!/usr/bin/env ruby

File.open("../README.md", 'r') do |f|
  f.each_line do |line|
    forbidden_words = ['Table of Contents', 'define', 'pragma']
    next if !line.start_with?("#") || forbidden_words.any? { |w| line =~ /#{w}/ }
    
    next if line.count("#") == 1 # No sh comments or 'top' headers
    next if line.count("#") >= 4 # No smaller headers
    
    title = line.gsub("#", "").strip
    href = title.gsub(" ", "-").downcase
    puts "  " * (line.count("#")-1) + "* [#{title}](\##{href})"
  end
end
