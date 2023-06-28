/* eslint-disable prettier/prettier */
import React from 'react';
import {StyleSheet} from 'react-native';
import {Dropdown as RNEDropdown} from 'react-native-element-dropdown';

export default function Dropdown({
  style,
  data,
  placeholder,
  value,
  onChange,
  ...props
}) {
  return (
    <RNEDropdown
      style={[styles.dropdown, style]}
      placeholderStyle={styles.placeholderStyle}
      selectedTextStyle={styles.selectedTextStyle}
      inputSearchStyle={styles.inputSearchStyle}
      iconStyle={styles.iconStyle}
      itemTextStyle={styles.selectedTextStyle}
      data={data}
      search
      maxHeight={300}
      labelField="label"
      valueField="value"
      placeholder={placeholder}
      searchPlaceholder="위치 검색..."
      value={value}
      onChange={onChange}
      {...props}
    />
  );
}

const styles = StyleSheet.create({
  dropdown: {
    // margin: 16,
    height: 60,
    width: '100%',
    borderRadius: 20,
    borderColor: '#0B537F',
    borderWidth: 2,
    padding: 12,
    shadowColor: '#000',
    shadowOffset: {
      width: 0,
      height: 1,
    },
    shadowOpacity: 0.2,
    shadowRadius: 1.41,

    elevation: 2,
  },
  icon: {
    marginRight: 5,
  },
  item: {
    padding: 17,
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    color: '#0B537F',
  },
  textItem: {
    flex: 1,
    fontSize: 16,
  },
  placeholderStyle: {
    fontSize: 16,
    color: '#0B537F',
  },
  selectedTextStyle: {
    fontSize: 16,
    color: '#0B537F',
  },
  iconStyle: {
    width: 20,
    height: 20,
  },
  inputSearchStyle: {
    height: 40,
    width: '70%',
    fontSize: 16,
    color: '#0B537F',
  },
});
