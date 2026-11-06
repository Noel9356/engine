# SPIR-V embedder (run with cmake -P). Reads a compiled .spv file and writes
# a C++ header with the words as an inline constexpr std::uint32_t array.
#
#   -DSPIRV_FILE=<path to .spv>      			(in)
#   -DHEADER_FILE=<path to .gen.hpp> 			(out)
#   -DSYMBOL_NAME=<array name>       			(in)
#   -DNAMESPACE=<namespace components; list> 	(in)
#   -DSOURCE_SHADER=<file name for the banner> 	(in)

if(NOT SPIRV_FILE OR NOT HEADER_FILE OR NOT SYMBOL_NAME)
	message(FATAL_ERROR
		"engine_spirv_embed: SPIRV_FILE, HEADER_FILE and SYMBOL_NAME are required"
	)
endif()

file(READ "${SPIRV_FILE}" spirvBytes HEX)
string(LENGTH "${spirvBytes}" byteCountHexChars)
math(EXPR wordCount "${byteCountHexChars} / 8")
if(wordCount EQUAL 0)
	message(FATAL_ERROR
		"engine_spirv_embed: ${SPIRV_FILE} is empty or not a multiple of 4 bytes"
	)
endif()

set(words)
set(wordIndex 0)
while(wordIndex LESS wordCount)
	math(EXPR firstChar "${wordIndex} * 8")
	string(SUBSTRING "${spirvBytes}" "${firstChar}" 8 byteHex)
	# SPIR-V is a little-endian word stream; compose each uint32 explicitly
	# instead of relying on the host endianness.
	string(SUBSTRING "${byteHex}" 6 2 byte3)
	string(SUBSTRING "${byteHex}" 4 2 byte2)
	string(SUBSTRING "${byteHex}" 2 2 byte1)
	string(SUBSTRING "${byteHex}" 0 2 byte0)
	string(TOUPPER "${byte3}${byte2}${byte1}${byte0}" wordHex)
	list(APPEND words "0x${wordHex}")
	math(EXPR wordIndex "${wordIndex} + 1")
endwhile()

# Sanity check: the magic number must be the first word.
list(GET words 0 magicWord)
if(NOT magicWord STREQUAL "0x07230203")
	message(FATAL_ERROR
		"engine_spirv_embed: ${SPIRV_FILE} does not start with the SPIR-V magic number"
	)
endif()

set(indent "    ")
set(bodyIndent "        ")
set(wordTokens)
foreach(word IN LISTS words)
	list(APPEND wordTokens "${word},")
endforeach()
set(bodyLines)
set(tokenCount)
list(LENGTH wordTokens tokenCount)
set(tokenIndex 0)
while(tokenIndex LESS tokenCount)
	set(line "${bodyIndent}")
	set(wordsOnLine 0)
	set(lineParts)
	while(wordsOnLine LESS 8 AND tokenIndex LESS tokenCount)
		list(GET wordTokens "${tokenIndex}" word)
		list(APPEND lineParts "${word}")
		math(EXPR tokenIndex "${tokenIndex} + 1")
		math(EXPR wordsOnLine "${wordsOnLine} + 1")
	endwhile()
	list(JOIN lineParts " " joinedLine)
	string(APPEND line "${joinedLine}")
	list(APPEND bodyLines "${line}")
endwhile()

set(header)
string(APPEND
	header
	"// GENERATED FILE - do not edit. Compiled from ${SOURCE_SHADER} by the\n"
)
string(APPEND
	header
	"// engine's shader build step (cmake/engine_module_compile_shaders.cmake).\n"
)
string(APPEND
	header
	"// A copy of this file may be committed next to the GLSL source for\n"
)
string(APPEND
	header
	"// toolchains without a shader compiler; builds with glslangValidator or\n"
)
string(APPEND
	header
	"// glslc regenerate this header into the build tree instead.\n"
)
string(APPEND header "#pragma once\n")
string(APPEND header "\n")
string(APPEND header "#include <cstdint>\n")
string(APPEND header "\n")
foreach(namespaceComponent IN LISTS NAMESPACE)
	string(APPEND header "namespace ${namespaceComponent}\n{\n")
endforeach()
string(APPEND header "\n")
string(APPEND
	header
	"${indent}inline constexpr std::uint32_t ${SYMBOL_NAME}[] = {\n"
)
foreach(bodyLine IN LISTS bodyLines)
	string(APPEND header "${bodyLine}\n")
endforeach()
string(APPEND header "${indent}};\n")
set(closingNamespaces "${NAMESPACE}")
list(REVERSE closingNamespaces)
foreach(namespaceComponent IN LISTS closingNamespaces)
	string(APPEND header "} // namespace ${namespaceComponent}\n")
endforeach()

file(WRITE "${HEADER_FILE}" "${header}")
