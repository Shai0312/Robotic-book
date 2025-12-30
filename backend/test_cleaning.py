#!/usr/bin/env python3
"""
Test script to see how the current text cleaner processes content.
"""

import requests
from bs4 import BeautifulSoup

# Fetch the main page
url = "https://robotic-book-murex.vercel.app/"
response = requests.get(url)
html_content = response.text

print("Original HTML length:", len(html_content))
print("\nFirst 1000 characters of HTML:")
print(html_content[:1000])
print("\n" + "="*50 + "\n")

# Current cleaning approach
soup = BeautifulSoup(html_content, 'html.parser')

# Remove script and style elements
for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
    script.decompose()

# Get text content
text = soup.get_text()

# Clean up the text (current approach)
lines = (line.strip() for line in text.splitlines())
chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
cleaned_text = ' '.join(chunk for chunk in chunks if chunk)

print("Current cleaning result length:", len(cleaned_text))
print("\nFirst 500 characters of cleaned text:")
print(cleaned_text[:500])
print("\n" + "="*50 + "\n")

# Improved cleaning approach
soup2 = BeautifulSoup(html_content, 'html.parser')

# Remove only the most problematic elements, preserve main content
for script in soup2(["script", "style"]):
    script.decompose()

# Keep main content elements but remove navigation and sidebars
for element in soup2(["nav", "header", "footer", "aside"]):
    # Only remove if they're not main content containers
    element.decompose()

# Get text with better preservation of structure
text2 = soup2.get_text(separator='\n')
lines2 = (line.strip() for line in text2.splitlines())
cleaned_text2 = '\n'.join(line for line in lines2 if line)

print("Improved cleaning result length:", len(cleaned_text2))
print("\nFirst 1000 characters of improved cleaned text:")
print(cleaned_text2[:1000])