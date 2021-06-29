function doGet(e){
  
    var ss = SpreadsheetApp.getActive();
  
    var sheet_name=e.parameter["id"];
    var sheet = ss.getSheetByName(e.parameter["id"]);
    var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];
    var lastRow = sheet.getLastRow();
    var ship_headers=["Order Number","Item","Quantity","Shipped Date and Time","City","Cost","Estimated Time of Delivery"];
    var disp_headers=["Order Number","Item","Quantity","Dispatch Date and Time","City","Cost"]
    var data="";
    var subj_data="";
    var cell = sheet.getRange('a1');
    var col = 0;
    var d=Utilities.formatDate(new Date(), SpreadsheetApp.getActive().getSpreadsheetTimeZone(), "dd/MM/yyyy hh:mm:ss a");
    
    for (i in headers){
  
      // loop through the headers and if a parameter name matches the header name insert the value
  
      if (headers[i] == "Timestamp")
      {
        val = d
      }
      else
      {
        val = e.parameter[headers[i]]
        
      }
  
      // append data to the last row
      cell.offset(lastRow, col).setValue(val);
      col++;
    }
    if (sheet_name=="OrdersDispatched"){
      subj_data=sheet_name.slice(6)+".";
      for(j in disp_headers){
       var header=disp_headers[j];
       if (header=="Order Number"){
         data=data+header+" "+":"+" "+e.parameter["Order Id"]+"<br>";
       }
       else if(header=="Quantity"){
         data=data+header+" "+":"+" "+e.parameter["Dispatch Quantity"]+"<br>";
       }
       else if(header=="Dispatch Date and Time"){
         data=data+header+" "+":"+" "+Utilities.formatDate(new Date(e.parameter[header]+"z"), "GMT", "EEE MMM dd YYYY, hh:mm:ss")+"<br>";
       }
       else{
       data=data+header+" "+":"+" "+e.parameter[header]+"<br>";
       }
      }
    }
    else if(sheet_name=="OrdersShipped"){
      subj_data=sheet_name.slice(6)+"!";
      for(j in ship_headers){
       var header=ship_headers[j];
       if (header=="Order Number"){
         data=data+header+" "+":"+" "+e.parameter["Order Id"]+"<br>";
       }
       else if(header=="Quantity"){
         data=data+header+" "+":"+" "+e.parameter["Shipped Quantity"]+"<br>";
       }
       else if(header=="Shipped Date and Time"){
         data=data+header+" "+":"+" "+Utilities.formatDate(new Date(e.parameter[header]+"z"), "GMT", "EEE MMM dd YYYY, hh:mm:ss")+"<br>";
       }
       else if(header=="Estimated Time of Delivery"){
         data=data+header+" "+":"+" "+Utilities.formatDate(new Date(e.parameter[header]+"T"+e.parameter["Shipped Date and Time"].slice(11)+"z"), "GMT", "EEE MMM dd YYYY, hh:mm:ss")+"<br>";
       }
       else{
       data=data+header+" "+":"+" "+e.parameter[header]+"<br>";
       }
      }
    }
    if(data.length){
    var recipient = "eyrc.vb.0000@gmail.com";   //eyrc mail id 
    var message = "Hello!"+"<br>"+"<br>"+"Your Order has been "+sheet_name.slice(6)+"."+"Conact us if you have any questions.We are here to help you"+"<br>"+"<br>"+"ORDER SUMMARY :"+"<br>"+"<br>"+ data ; 
    MailApp.sendEmail({to:recipient, subject:" Your Order Is "+subj_data,htmlBody:message});
    }
    
    return ContentService.createTextOutput('success');
   
  }
  